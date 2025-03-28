#include <unitree_arm_sdk/control/unitreeArm.h>

int main() {
  UNITREE_ARM::unitreeArm arm_sdk_(false);
  arm_sdk_.sendRecvThread->start();
  arm_sdk_.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
  arm_sdk_.backToStart();
  Eigen::Matrix<double, 6, 1> goal_pose;

  //   goal_pose << 0, 0, 0, 0.56, 0., 0.05;
  goal_pose << 0.5, 0.1, 0.1, 0.5, -0.2, 0.5;
  std::cout << "MoveJ begins" << std::endl;
  arm_sdk_.MoveJ(goal_pose, 0.6);
  std::cout << "MoveJ ends" << std::endl;
  arm_sdk_.backToStart();
  arm_sdk_.sendRecvThread->shutdown();
}
