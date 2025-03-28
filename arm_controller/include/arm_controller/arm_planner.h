#ifndef ARM_PLANNER_H_
#define ARM_PLANNER_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
namespace arm_controller {

class ArmPlanner {
 public:
  ArmPlanner(const ros::NodeHandle &nh);
  ~ArmPlanner();

  void plan(std::vector<Eigen::Matrix4d> &joint_trajectory);

  void subsAndPubs() {}

 protected:
  const std::string kPlanningGroupName{"z1_arm"};
  ros::NodeHandle nh_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_ptr_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup *joint_model_group_;
};

}  // namespace arm_controller
#endif
