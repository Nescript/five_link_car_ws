#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

namespace five_link_car_controller {

class FiveLinkCarController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
public:
  FiveLinkCarController() = default;
  ~FiveLinkCarController() = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  hardware_interface::JointHandle link4_joint_, link1_joint_;
  control_toolbox::Pid pid_link4_pos_, pid_link1_pos_;
};
} //namespace 