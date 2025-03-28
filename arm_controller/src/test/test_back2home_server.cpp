#include <ros/ros.h>

#include <cstdlib>

#include "arm_controller_srvs/BackToHome.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "plan_client");

  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<arm_controller_srvs::BackToHome>("back_to_home");
  arm_controller_srvs::BackToHome srv;
  srv.request.back_to_home = true;
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
