#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <cmath>

namespace five_link_car_controller {

/**
 * @brief 五连杆并联机构控制器
 *
 * 运动学坐标系定义（与 five_link_check.py 一致）:
 *   - 关节A（link1_joint）位于原点 (0, 0)
 *   - 关节D（link4_joint）位于 (l5, 0)
 *   - x 轴从 A 指向 D（水平方向）
 *   - y 轴向上（机构延伸方向）
 *   - 末端P为 link2 与 link3 的连接点
 *
 * URDF关节角度与运动学角度的映射:
 *   - phi1 = theta1_urdf + pi
 *   - phi4 = theta4_urdf
 */
class FiveLinkCarController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
  FiveLinkCarController() = default;
  ~FiveLinkCarController() = default;

  bool init(hardware_interface::PositionJointInterface *position_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time &time) override;

  /**
   * @brief 正运动学：输入两关节电机角度，输出末端坐标
   * @param theta1 link1_joint 角度 (URDF关节角, rad)
   * @param theta4 link4_joint 角度 (URDF关节角, rad)
   * @param px 输出末端 x 坐标 (运动学坐标系, m)
   * @param py 输出末端 y 坐标 (运动学坐标系, m)
   * @return true 计算成功
   */
  bool forwardKinematics(double theta1, double theta4, double &px, double &py);

  /**
   * @brief 逆运动学：输入末端坐标，输出两关节电机角度
   * @param px 末端 x 坐标 (运动学坐标系, m)
   * @param py 末端 y 坐标 (运动学坐标系, m)
   * @param theta1 输出 link1_joint 角度 (URDF关节角, rad)
   * @param theta4 输出 link4_joint 角度 (URDF关节角, rad)
   * @return true 计算成功
   */
  bool inverseKinematics(double px, double py, double &theta1, double &theta4);

  /**
   * @brief 正弦轨迹规划：每周期生成当前时刻的目标末端位置
   *
   * 在矩形区域内生成正弦运动路径：
   *   - x 方向线性来回（三角波），到达两端自动反向
   *   - y 方向为 x 的正弦函数，形成正弦曲线路径
   *
   * @param time 当前时间
   * @param target_x 输出目标 x 坐标
   * @param target_y 输出目标 y 坐标
   */
  void sinusoidalTrajectory(const ros::Time &time, double &target_x, double &target_y);

private:
  hardware_interface::JointHandle link4_joint_, link1_joint_;
  control_toolbox::Pid pid_link4_pos_, pid_link1_pos_;

  // 五连杆杆长参数（从参数服务器加载）
  double l1_{0}, l2_{0}, l3_{0}, l4_{0}, l5_{0};

  // 轨迹规划参数
  double traj_x_center_{0};      // 轨迹中心 x
  double traj_y_center_{0};      // 轨迹中心 y
  double traj_x_amplitude_{0};   // x 方向振幅
  double traj_y_amplitude_{0};   // y 方向振幅
  double traj_frequency_{0};     // 运动频率 (Hz)
  int    traj_sine_periods_{0};  // y 方向正弦半周期数

  ros::Publisher pub_target_pose_;
  ros::Publisher pub_current_pose_;
  // 运行状态
  ros::Time start_time_;
  double target_x_{0}, target_y_{0};
};

} // namespace five_link_car_controller
