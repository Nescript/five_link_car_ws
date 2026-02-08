#include <ros/ros.h>
#include "five_link_car_controller/five_link_car_controller.hpp"

namespace five_link_car_controller {

bool FiveLinkCarController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  try {
    link1_joint_ = effort_joint_interface->getHandle("link1_joint");
    link4_joint_ = effort_joint_interface->getHandle("link4_joint");
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("Could not get joint handles: %s", e.what());
    return false;
  }
  

  return true;
}

void FiveLinkCarController::update(const ros::Time& time, const ros::Duration& period) {
  // Update code here
}

void FiveLinkCarController::starting(const ros::Time& time) {
  // Starting code here
}

void FiveLinkCarController::stopping(const ros::Time& time) {
  // Stopping code here
}

} //namespace five_link_car_controller