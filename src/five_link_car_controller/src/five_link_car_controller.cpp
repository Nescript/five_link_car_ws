#include <ros/ros.h>
#include "five_link_car_controller/five_link_car_controller.hpp"
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <algorithm>

namespace five_link_car_controller {

bool FiveLinkCarController::init(hardware_interface::PositionJointInterface *position_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  try {
    link1_joint_ = position_joint_interface->getHandle("link1_joint");
    link4_joint_ = position_joint_interface->getHandle("link4_joint");
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("Could not get joint handles: " << e.what());
    return false;
  }

  if (!pid_link4_pos_.init(ros::NodeHandle(controller_nh, "pid_link4"))) {
    ROS_ERROR("Failed to init link4 pid");
    return false;
  }
  if (!pid_link1_pos_.init(ros::NodeHandle(controller_nh, "pid_link1"))) {
    ROS_ERROR("Failed to init link1 pid");
    return false;
  }

  if (!controller_nh.getParam("l1", l1_) ||
      !controller_nh.getParam("l2", l2_) ||
      !controller_nh.getParam("l3", l3_) ||
      !controller_nh.getParam("l4", l4_) ||
      !controller_nh.getParam("l5", l5_)) {
    ROS_ERROR("Failed to load link lengths from parameter server");
    return false;
  }
  ROS_INFO("Five-link lengths: l1=%.3f, l2=%.3f, l3=%.3f, l4=%.3f, l5=%.3f",
           l1_, l2_, l3_, l4_, l5_);

  controller_nh.param("traj_x_center",      traj_x_center_,     l5_ / 2.0);  // 基座中心
  controller_nh.param("traj_y_center",      traj_y_center_,     0.22);        // 工作区中部
  controller_nh.param("traj_x_amplitude",   traj_x_amplitude_,  0.03);
  controller_nh.param("traj_y_amplitude",   traj_y_amplitude_,  0.02);
  controller_nh.param("traj_frequency",     traj_frequency_,    0.5);         // 0.5 Hz
  controller_nh.param("traj_sine_periods",  traj_sine_periods_, 2);           // 2个正弦半周期

  ROS_INFO("Trajectory: center=(%.3f,%.3f) amp=(%.3f,%.3f) freq=%.2f periods=%d",
           traj_x_center_, traj_y_center_,
           traj_x_amplitude_, traj_y_amplitude_,
           traj_frequency_, traj_sine_periods_);

  return true;
}

bool FiveLinkCarController::forwardKinematics(double phi_1, double phi_4,
                                              double &px, double &py) {
  // to do: make sure phi_1 is the same with the model
  phi_1 = phi_1 + M_PI;
  // 
  double x_b = l1_ * cos(phi_1);
  double y_b = l1_ * sin(phi_1);
  double x_d = l5_ + l4_ * cos(phi_4);
  double y_d = l4_ * sin(phi_4);
  double l_bd = sqrt((x_d - x_b) * (x_d - x_b) + (y_d - y_b) * (y_d - y_b));
  
  double a = 2 * (x_b - x_d) * l2_;
  double b = 2 * (y_b - y_d) * l2_;
  double c = l3_ * l3_ - l2_ * l2_ - l_bd * l_bd;
  double discriminant = a * a + b * b - c * c;
  if (discriminant < 0) {
    ROS_WARN("FK: no solution for phi_2");
    return false;
  }
  double sqrt_discriminant = sqrt(discriminant);
  double phi_2 = 2.0 * atan2((b + sqrt_discriminant), (a + c));
  
  px = x_b + l2_ * cos(phi_2);
  py = y_b + l2_ * sin(phi_2);
  
  return true;
}

// ========================== 逆运动学 ==========================
bool FiveLinkCarController::inverseKinematics(double px, double py,
                                              double &theta1, double &theta4) {
  double x_c = px;
  double y_c = py;
  double a = 2 * x_c * l1_;
  double b = 2 * y_c * l1_;
  double c = x_c * x_c + y_c * y_c + l1_ * l1_ - l2_ * l2_;
  double discriminant = a * a + b * b - c * c;
  if (discriminant < 0) {
    ROS_WARN("IK: no solution for theta1");
    return false;
  }
  double sqrt_discriminant = sqrt(discriminant);
  double phi1 = 2.0 * atan2((b + sqrt_discriminant), (a + c));
  
  double x_d = l5_ + l4_ * cos(phi1);
  double y_d = l4_ * sin(phi1);
  double dxr = px - x_d;
  double dyr = py - y_d;
  double dist_sq_r = dxr * dxr + dyr * dyr;
  if (dist_sq_r < 1e-10) {
    ROS_WARN("IK: target too close to joint D");
    return false;
  }
  double cos_beta = (l4_ * l4_ + dist_sq_r - l3_ * l3_) / (2.0 * l4_ * sqrt(dist_sq_r));
  if (cos_beta < -1.001 || cos_beta > 1.001) {
    ROS_WARN("IK: right chain unreachable (cos_beta=%.4f)", cos_beta);
    return false;
  }
  cos_beta = std::max(-1.0, std::min(1.0, cos_beta));
  
  double gamma_r = atan2(dyr, dxr);
  double alpha_r = acos(cos_beta);
  double phi4 = gamma_r - alpha_r;

  theta1 = phi1 - M_PI;
  theta4 = -phi4;
  ROS_INFO("Now pose: theta1=%.4f, theta4=%.4f", theta1, theta4);
  return true;
}

// ========================== 正弦轨迹规划 ==========================
void FiveLinkCarController::sinusoidalTrajectory(const ros::Time &time,
                                                  double &target_x, double &target_y) {
  /*
   * 在矩形区域内生成正弦运动轨迹:
   *   矩形范围:
   *     x ∈ [x_center - x_amp, x_center + x_amp]
   *     y ∈ [y_center - y_amp, y_center + y_amp]
   *
   *   x 方向: 三角波（线性来回），到端点自动反向
   *   y 方向: sin(n * π * s)，s 为归一化 x 位置 [-1, 1]
   *
   * 效果: 末端在矩形内沿正弦曲线来回移动
   */
  double elapsed = (time - start_time_).toSec();

  // 归一化三角波 s ∈ [-1, 1]，周期 T = 1 / traj_frequency_
  double phase = elapsed * traj_frequency_;
  double s = 2.0 * fabs(fmod(fabs(phase), 2.0) - 1.0) - 1.0;

  // x: 线性来回
  target_x = traj_x_center_ + traj_x_amplitude_ * s;

  // y: 正弦曲线（n 个半周期）
  target_y = traj_y_center_ + traj_y_amplitude_ * sin(traj_sine_periods_ * M_PI * s);
}

// ========================== update ==========================
void FiveLinkCarController::update(const ros::Time &time, const ros::Duration &period) {
  // 1. 轨迹规划 → 末端目标位置（运动学坐标系）
  double tgt_x, tgt_y;
  sinusoidalTrajectory(time, tgt_x, tgt_y);

  // 2. 逆运动学 → 目标关节角度
  double tgt_theta1, tgt_theta4;
  if (!inverseKinematics(tgt_x, tgt_y, tgt_theta1, tgt_theta4)) {
    ROS_WARN_THROTTLE(1.0, "IK failed for target (%.4f, %.4f)", tgt_x, tgt_y);
    return;
  }

  // 3. 读取当前关节角度
  double cur_theta1 = link1_joint_.getPosition();
  double cur_theta4 = link4_joint_.getPosition();

  // 4. PID 误差计算（可用于调试或切换为力矩控制时使用）
  double error1 = tgt_theta1 - cur_theta1;
  double error4 = tgt_theta4 - cur_theta4;
  pid_link1_pos_.computeCommand(error1, period);
  pid_link4_pos_.computeCommand(error4, period);

  // 5. 位置接口：直接发送目标角度
  link1_joint_.setCommand(tgt_theta1);
  link4_joint_.setCommand(tgt_theta4);
}

// ========================== starting ==========================
void FiveLinkCarController::starting(const ros::Time &time) {
  start_time_ = time;
  pid_link1_pos_.reset();
  pid_link4_pos_.reset();

  // 读取当前关节位置并正运动学解算初始末端坐标
  double cur_theta1 = link1_joint_.getPosition();
  double cur_theta4 = link4_joint_.getPosition();
  if (forwardKinematics(cur_theta1, cur_theta4, target_x_, target_y_)) {
    ROS_INFO("FiveLinkCarController started. Initial end-effector: (%.4f, %.4f)",
             target_x_, target_y_);
  } else {
    ROS_WARN("FiveLinkCarController started. FK failed for initial pose.");
  }
}

// ========================== stopping ==========================
void FiveLinkCarController::stopping(const ros::Time &time) {
  ROS_INFO("FiveLinkCarController stopped.");
}

} // namespace five_link_car_controller

// ========================== 注册 ros_control 插件 ==========================
PLUGINLIB_EXPORT_CLASS(five_link_car_controller::FiveLinkCarController,
                       controller_interface::ControllerBase)