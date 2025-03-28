#include <ros/ros.h>

#include <cstdlib>

#include "arm_controller_srvs/Plan.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "plan_client");

  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<arm_controller_srvs::Plan>("plan");
  arm_controller_srvs::Plan srv;
  srv.request.target_pose.position.x = 0.2;
  srv.request.target_pose.position.y = 0.3;
  srv.request.target_pose.position.z = 0.2;
  srv.request.target_pose.orientation.w = 1;
  srv.request.target_pose.orientation.x = 0;
  srv.request.target_pose.orientation.y = 0;
  srv.request.target_pose.orientation.z = 0;
  if (client.call(srv)) {
    std::cout << "Response: ";
    if (srv.response.call_success) {
      std::cout << "now plan" << std::endl;
    } else {
      std::cout << "plan failed" << std::endl;
    }
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  return 0;
}
