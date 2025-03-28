#include <ros/ros.h>

#include "arm_controller/arm_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_moveit_planner");
  ros::NodeHandle nh;
  arm_controller::ArmPlanner planner(nh);

}
