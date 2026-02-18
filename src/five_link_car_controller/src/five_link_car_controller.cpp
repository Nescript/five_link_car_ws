#include <ros/ros.h>
#include "five_link_car_controller/five_link_car_controller.hpp"
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <algorithm>

namespace five_link_car_controller {

// ========================== init ==========================
bool FiveLinkCarController::init(hardware_interface::PositionJointInterface *position_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  // ---------- 获取关节句柄 ----------
  try {
    link1_joint_ = position_joint_interface->getHandle("link1_joint");
    link4_joint_ = position_joint_interface->getHandle("link4_joint");
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("Could not get joint handles: " << e.what());
    return false;
  }

  // ---------- 初始化 PID 控制器 ----------
  if (!pid_link4_pos_.init(ros::NodeHandle(controller_nh, "pid_link4"))) {
    ROS_ERROR("Failed to init link4 pid");
    return false;
  }
  if (!pid_link1_pos_.init(ros::NodeHandle(controller_nh, "pid_link1"))) {
    ROS_ERROR("Failed to init link1 pid");
    return false;
  }

  // ---------- 从参数服务器加载杆长 ----------
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

  // ---------- 加载轨迹规划参数（带默认值）----------
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

// ========================== 正运动学 ==========================
bool FiveLinkCarController::forwardKinematics(double theta1, double theta4,
                                              double &px, double &py) {
  /*
   * 运动学坐标系: A=(0,0), D=(l5,0)
   *   phi1 = theta1_urdf + pi   (link1 运动学角)
   *   phi4 = theta4_urdf         (link4 运动学角)
   *   B = A + l1*(cos phi1, sin phi1)
   *   E = D + l4*(cos phi4, sin phi4)
   *   P 为以 B 为圆心半径 l2 与以 E 为圆心半径 l3 两圆的交点
   */
  double phi1 = theta1 + M_PI;
  double phi4 = theta4;

  // link1 末端 B、link4 末端 E
  double bx = l1_ * cos(phi1);
  double by = l1_ * sin(phi1);
  double ex = l5_ + l4_ * cos(phi4);
  double ey = l4_ * sin(phi4);

  // ---- 两圆交点 ----
  double dx = ex - bx;
  double dy = ey - by;
  double d  = sqrt(dx * dx + dy * dy);

  if (d > l2_ + l3_ + 1e-6 || d < fabs(l2_ - l3_) - 1e-6 || d < 1e-10) {
    ROS_WARN("FK: no solution (d=%.4f, valid=[%.4f, %.4f])",
             d, fabs(l2_ - l3_), l2_ + l3_);
    return false;
  }

  double a    = (l2_ * l2_ - l3_ * l3_ + d * d) / (2.0 * d);
  double h_sq = l2_ * l2_ - a * a;
  if (h_sq < 0.0) h_sq = 0.0;
  double h = sqrt(h_sq);

  // 中间点 M = B + (a/d)*(E-B)
  double mx = bx + a * dx / d;
  double my = by + a * dy / d;

  // 两个候选交点
  double p1x = mx + h * (-dy) / d;
  double p1y = my + h * (dx)  / d;
  double p2x = mx - h * (-dy) / d;
  double p2y = my - h * (dx)  / d;

  // 选择 y 较大的解（机构正常工作区域在基座上方）
  if (p1y >= p2y) { px = p1x; py = p1y; }
  else            { px = p2x; py = p2y; }

  return true;
}

// ========================== 逆运动学 ==========================
bool FiveLinkCarController::inverseKinematics(double px, double py,
                                              double &theta1, double &theta4) {
  /*
   * 左侧支链 A-B-P:  A=(0,0), |AB|=l1, |BP|=l2
   *   phi1 = gamma_left + alpha_left
   * 右侧支链 D-E-P:  D=(l5,0), |DE|=l4, |EP|=l3
   *   phi4 = gamma_right - alpha_right
   */

  // ---- 左侧支链 ----
  double dist_sq_l = px * px + py * py;
  double dist_l    = sqrt(dist_sq_l);
  if (dist_l < 1e-10) {
    ROS_WARN("IK: target too close to joint A");
    return false;
  }
  double cos_alpha = (l1_ * l1_ + dist_sq_l - l2_ * l2_) / (2.0 * l1_ * dist_l);
  if (cos_alpha < -1.001 || cos_alpha > 1.001) {
    ROS_WARN("IK: left chain unreachable (cos_alpha=%.4f)", cos_alpha);
    return false;
  }
  cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));

  double gamma_l  = atan2(py, px);
  double alpha_l  = acos(cos_alpha);
  double phi1     = gamma_l + alpha_l;

  // ---- 右侧支链 ----
  double dxr      = px - l5_;
  double dist_sq_r = dxr * dxr + py * py;
  double dist_r    = sqrt(dist_sq_r);
  if (dist_r < 1e-10) {
    ROS_WARN("IK: target too close to joint D");
    return false;
  }
  double cos_beta = (l4_ * l4_ + dist_sq_r - l3_ * l3_) / (2.0 * l4_ * dist_r);
  if (cos_beta < -1.001 || cos_beta > 1.001) {
    ROS_WARN("IK: right chain unreachable (cos_beta=%.4f)", cos_beta);
    return false;
  }
  cos_beta = std::max(-1.0, std::min(1.0, cos_beta));

  double gamma_r  = atan2(py, dxr);
  double alpha_r  = acos(cos_beta);
  double phi4     = gamma_r - alpha_r;

  // ---- 运动学角度 → URDF 关节角度 ----
  theta1 = phi1 - M_PI;
  theta4 = phi4;

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