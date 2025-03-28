#include <ros/ros.h>

#include <cstdlib>

#include "arm_controller/geometry_utils.h"
#include "arm_controller_srvs/CheckPoseInWorkspace.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_good_cmd_pose");

  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<arm_controller_srvs::CheckPoseInWorkspace>(
          "is_in_workspace");
  double depth;
  const double kAngleRange{M_PI_2};
  double angle{0.};
  Eigen::Vector4d kQuatBias{std::cos(M_PI_4), std::sin(M_PI_4), 0, 0};
  Eigen::Vector4d quat, quat_W;
  int iter_num{1000};
  for (int depth_i{0}; depth_i < 800; ++depth_i) {
    depth = 0.4 + 0.001 * depth_i;
    angle = 0;
    for (int i = 0; i < iter_num; ++i) {
      arm_controller_srvs::CheckPoseInWorkspace srv;
      angle = kAngleRange * i / iter_num;
      double l = depth / (std::cos(angle) + std::sin(angle));
      srv.request.target_pose.position.x = l * std::cos(angle);
      srv.request.target_pose.position.y = l * std::sin(angle);
      srv.request.target_pose.position.z = 0.05;
      quat << std::cos(M_PI / 8), 0., std::sin(M_PI / 8), 0.;
      arm_controller::quatProduct(kQuatBias, quat, quat_W);
      // srv.request.target_pose.orientation.w = std::cos(M_PI / 8);
      // srv.request.target_pose.orientation.x = 0;
      // srv.request.target_pose.orientation.y = 0;
      // srv.request.target_pose.orientation.z = -std::sin(M_PI / 8);
      srv.request.target_pose.orientation.w = quat_W[0];
      srv.request.target_pose.orientation.x = quat_W[1];
      srv.request.target_pose.orientation.y = quat_W[2];
      srv.request.target_pose.orientation.z = quat_W[3];
      if (!client.call(srv)) {
        std::cout << "Call failed" << std::endl;
      };
      if (srv.response.is_in_workspace) {
        std::cout << "Find success! Depth: " << depth << " Length: " << l
                  << " Angle: " << angle << " Position: ["
                  << srv.request.target_pose.position.x << ","
                  << srv.request.target_pose.position.y << ","
                  << srv.request.target_pose.position.z
                  << "], Orientation(xyzw): ["
                  << srv.request.target_pose.orientation.x << ","
                  << srv.request.target_pose.orientation.y << ","
                  << srv.request.target_pose.orientation.z << ","
                  << srv.request.target_pose.orientation.w << "]" << std::endl;
      }
    }
  }
  return 0;
}
