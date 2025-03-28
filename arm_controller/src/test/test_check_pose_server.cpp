#include <ros/ros.h>

#include <cstdlib>

#include "arm_controller_srvs/CheckPoseInWorkspace.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "plan_client");

  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<arm_controller_srvs::CheckPoseInWorkspace>("is_in_workspace");
  arm_controller_srvs::CheckPoseInWorkspace srv;
  srv.request.target_pose.position.x = 0.15;
  srv.request.target_pose.position.y = 0.15;
  srv.request.target_pose.position.z = 0.22;
  srv.request.target_pose.orientation.w = 1;
  srv.request.target_pose.orientation.x = 0;
  srv.request.target_pose.orientation.y = 0;
  srv.request.target_pose.orientation.z = 0;
  if (client.call(srv)) {
    std::cout << "Response: ";
    if (srv.response.is_in_workspace) {
      std::cout << "the target pose is in workspace" << std::endl;
    } else {
      std::cout << "the target pose is not in workspace" << std::endl;
    }
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  return 0;
}
