#include "arm_controller/arm_planner.h"

namespace arm_controller {

ArmPlanner::ArmPlanner(const ros::NodeHandle& nh) : nh_(nh) {
  move_group_ptr_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          kPlanningGroupName);
  joint_model_group_ = move_group_ptr_->getCurrentState()->getJointModelGroup(
      kPlanningGroupName);
}

ArmPlanner::~ArmPlanner() {}

void ArmPlanner::plan(std::vector<Eigen::Matrix4d>& joint_trajectory) {}

}  // namespace arm_controller
