#include "arm_controller/kinematics.h"

#include "arm_controller/geometry_utils.h"

namespace arm_controller {

KinematicChain::KinematicChain(
    const std::vector<Eigen::Vector3d>& joint_axis,
    const std::vector<Eigen::Matrix3d>& joint_init_rot,
    const std::vector<Eigen::Vector3d>& joint_init_pos,
    const Eigen::Ref<const Eigen::Vector3d>& end_effector_pos_P,
    const Eigen::Ref<const Eigen::VectorXd>& joint_init_angles,
    const Eigen::Ref<const Eigen::MatrixXd>& joint_angle_limits)
    : joint_init_angles_(joint_init_angles),
      joint_angles_limits_(joint_angle_limits) {
  num_joints_ = joint_axis.size();
  assert(num_joints_ == joint_init_pos.size());
  assert(num_joints_ == joint_init_rot.size());
  assert(num_joints_ == joint_init_angles.size());
  assert(num_joints_ == joint_init_angles.size());
  assert(num_joints_ == joint_angle_limits.rows());
  // forward kinematic params initialization
  init_trans_mat_.setIdentity();
  for (int i = 0; i < num_joints_; ++i) {
    Eigen::Matrix4d trans_mat = Eigen::Matrix4d::Identity();
    trans_mat.block<3, 3>(0, 0) = joint_init_rot[i];
    trans_mat.block<3, 1>(0, 3) = joint_init_pos[i];
    init_trans_mat_ = init_trans_mat_ * trans_mat;
    Eigen::VectorXd axis(6);
    axis.setZero();
    axis.head(3) = joint_axis[i];
    screw_axis_homo_W_.emplace_back(
        adjointMapToTwistHomo(init_trans_mat_, axis));
  }
  init_trans_mat_.block<3, 1>(0, 3) =
      init_trans_mat_.block<3, 1>(0, 3) +
      init_trans_mat_.block<3, 3>(0, 0) * end_effector_pos_P;
  init_trans_mat_inv_.setIdentity();
  init_trans_mat_inv_.block<3, 3>(0, 0) =
      init_trans_mat_.block<3, 3>(0, 0).transpose();
  init_trans_mat_inv_.block<3, 1>(0, 3) =
      -init_trans_mat_inv_.block<3, 3>(0, 0) *
      init_trans_mat_.block<3, 1>(0, 3);
  for (int i = 0; i < num_joints_; ++i) {
    screw_axis_homo_B_.emplace_back(adjointMapTwistHomoToTwistHomo(
        init_trans_mat_inv_, screw_axis_homo_W_[i]));
  }
  // inverse kinematic solver params initialization
  kJointIdentityMat_.setIdentity(num_joints_, num_joints_);
  error_wight_.setOnes(6);
}

void KinematicChain::forwardKinematic(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
    Eigen::Ref<Eigen::Matrix4d> target_pose) {
  assert(joint_angles.size() == num_joints_);
  target_pose.setIdentity();
  for (int i = 0; i < num_joints_; ++i) {
    target_pose =
        target_pose * se3ToTransMat(screw_axis_homo_W_[i], joint_angles[i]);
  }
  target_pose = target_pose * init_trans_mat_;
}

Eigen::Matrix4d KinematicChain::forwardKinematic(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
  assert(joint_angles.size() == num_joints_);
  Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
  for (int i = 0; i < num_joints_; ++i) {
    target_pose =
        target_pose * se3ToTransMat(screw_axis_homo_W_[i], joint_angles[i]);
  }
  return target_pose * init_trans_mat_;
}

void KinematicChain::randomForwardKinematic(
    Eigen::Ref<Eigen::Matrix4d> target_pose) {
  Eigen::VectorXd joint_angles(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i) {
    joint_angles[i] = std::uniform_real_distribution<double>(
        joint_angles_limits_(i, 0), joint_angles_limits_(i, 1))(gen_);
  }
  forwardKinematic(joint_angles, target_pose);
}

Eigen::Matrix4d KinematicChain::randomForwardKinematic() {
  Eigen::VectorXd joint_angles(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i) {
    joint_angles[i] = std::uniform_real_distribution<double>(
        joint_angles_limits_(i, 0), joint_angles_limits_(i, 1))(gen_);
  }
  return forwardKinematic(joint_angles);
}

void KinematicChain::forwardKinematicWithSpatialJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
    Eigen::Ref<Eigen::Matrix4d> target_pose,
    Eigen::Ref<Eigen::MatrixXd> spatial_jacobian) {
  assert(joint_angles.size() == num_joints_);
  target_pose.setIdentity();
  for (int i = 0; i < num_joints_; ++i) {
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(target_pose, screw_axis_homo_W_[i]),
        spatial_jacobian.col(i));
    target_pose =
        target_pose * se3ToTransMat(screw_axis_homo_W_[i], joint_angles[i]);
  }
  target_pose = target_pose * init_trans_mat_;
}

Eigen::Matrix4d KinematicChain::forwardKinematicWithSpatialJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
    Eigen::Ref<Eigen::MatrixXd> spatial_jacobian) {
  assert(joint_angles.size() == num_joints_);
  Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
  for (int i = 0; i < num_joints_; ++i) {
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(target_pose, screw_axis_homo_W_[i]),
        spatial_jacobian.col(i));
    target_pose =
        target_pose * se3ToTransMat(screw_axis_homo_W_[i], joint_angles[i]);
  }
  return target_pose * init_trans_mat_;
}

void KinematicChain::forwardKinematicWithBodyJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
    Eigen::Ref<Eigen::Matrix4d> target_pose,
    Eigen::Ref<Eigen::MatrixXd> body_jacobian) {
  assert(joint_angles.size() == num_joints_);
  target_pose.setIdentity();
  Eigen::Matrix4d target_pose_inv = Eigen::Matrix4d::Identity();
  for (int i = num_joints_ - 1; i >= 0; --i) {
    target_pose_inv.block<3, 3>(0, 0) =
        target_pose.block<3, 3>(0, 0).transpose();
    target_pose_inv.block<3, 1>(0, 3) =
        -target_pose_inv.block<3, 3>(0, 0) * target_pose.block<3, 1>(0, 3);
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(target_pose_inv, screw_axis_homo_B_[i]),
        body_jacobian.col(i));
    target_pose =
        se3ToTransMat(screw_axis_homo_B_[i], joint_angles[i]) * target_pose;
  }
  target_pose = init_trans_mat_ * target_pose;
}

Eigen::Matrix4d KinematicChain::forwardKinematicWithBodyJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
    Eigen::Ref<Eigen::MatrixXd> body_jacobian) {
  assert(joint_angles.size() == num_joints_);
  Eigen::Matrix4d trans_mat = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d trans_mat_inv = Eigen::Matrix4d::Identity();
  for (int i = num_joints_ - 1; i >= 0; --i) {
    trans_mat_inv.block<3, 3>(0, 0) = trans_mat.block<3, 3>(0, 0).transpose();
    trans_mat_inv.block<3, 1>(0, 3) =
        -trans_mat_inv.block<3, 3>(0, 0) * trans_mat.block<3, 1>(0, 3);
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(trans_mat_inv, screw_axis_homo_B_[i]),
        body_jacobian.col(i));
    trans_mat =
        se3ToTransMat(screw_axis_homo_B_[i], joint_angles[i]) * trans_mat;
  }
  return init_trans_mat_ * trans_mat;
}

void KinematicChain::calculateSpatialJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
    Eigen::Ref<Eigen::MatrixXd> spatial_jacobian) {
  assert(joint_angles.size() == num_joints_);
  Eigen::Matrix4d trans_mat_W = Eigen::Matrix4d::Identity();
  for (int i = 0; i < num_joints_; ++i) {
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(trans_mat_W, screw_axis_homo_W_[i]),
        spatial_jacobian.col(i));
    trans_mat_W =
        trans_mat_W * se3ToTransMat(screw_axis_homo_W_[i], joint_angles[i]);
  }
}

Eigen::MatrixXd KinematicChain::calculateSpatialJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
  assert(joint_angles.size() == num_joints_);
  Eigen::MatrixXd spatial_jacobian(6, num_joints_);
  Eigen::Matrix4d trans_mat_W = Eigen::Matrix4d::Identity();
  for (int i = 0; i < num_joints_; ++i) {
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(trans_mat_W, screw_axis_homo_W_[i]),
        spatial_jacobian.col(i));
    trans_mat_W =
        trans_mat_W * se3ToTransMat(screw_axis_homo_W_[i], joint_angles[i]);
  }
  return spatial_jacobian;
}

void KinematicChain::calculateBodyJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
    Eigen::Ref<Eigen::MatrixXd> body_jacobian) {
  assert(joint_angles.size() == num_joints_);
  Eigen::Matrix4d trans_mat_B = Eigen::Matrix4d::Identity();
  for (int i = num_joints_ - 1; i >= 0; --i) {
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(trans_mat_B, screw_axis_homo_B_[i]),
        body_jacobian.col(i));
    trans_mat_B =
        trans_mat_B * se3ToTransMat(-screw_axis_homo_B_[i], joint_angles[i]);
  }
}

Eigen::MatrixXd KinematicChain::calculateBodyJacobian(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
  assert(joint_angles.size() == num_joints_);
  Eigen::MatrixXd body_jacobian(6, num_joints_);
  Eigen::Matrix4d trans_mat_B = Eigen::Matrix4d::Identity();
  for (int i = num_joints_ - 1; i >= 0; --i) {
    twistHomoToTwist(
        adjointMapTwistHomoToTwistHomo(trans_mat_B, screw_axis_homo_B_[i]),
        body_jacobian.col(i));
    trans_mat_B =
        trans_mat_B * se3ToTransMat(-screw_axis_homo_B_[i], joint_angles[i]);
  }
  return body_jacobian;
}

bool KinematicChain::inverseKinematic(
    const Eigen::Ref<const Eigen::Matrix3d>& target_rot,
    const Eigen::Ref<const Eigen::Vector3d>& target_pos,
    const Eigen::Ref<const Eigen::VectorXd>& init_guess,
    Eigen::Ref<Eigen::VectorXd> joint_pos, double& error,
    const unsigned int max_iterations, const double& eps) {
  assert(init_guess.size() == num_joints_);
  bool success = false;
  Eigen::MatrixXd body_jacobian(6, num_joints_);
  Eigen::Matrix4d target_pose, pose;
  // Eigen::Matrix4d pose_inv;
  Eigen::VectorXd error_twist(6);
  Eigen::Array<bool, Eigen::Dynamic, 1> joint_reach_limit =
      Eigen::Array<bool, Eigen::Dynamic, 1>::Zero(num_joints_);
  target_pose.block<3, 3>(0, 0) = target_rot;
  target_pose.block<3, 1>(0, 3) = target_pos;
  joint_pos = init_guess;
  error = 0.0;
  for (unsigned int iter = 0; iter != max_iterations; ++iter) {
    forwardKinematicWithBodyJacobian(joint_pos, pose, body_jacobian);
    // pose_inv.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
    // pose_inv.block<3, 1>(0, 3) =
    //     -pose_inv.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3);
    // transMatToTwist(pose_inv * target_pose, error_twist);
    rotToVec(pose.block<3, 3>(0, 0).transpose() * target_rot,
             error_twist.head<3>());
    error_twist.tail<3>() = pose.block<3, 3>(0, 0).transpose() *
                            (target_pos - pose.block<3, 1>(0, 3));
    error = 0.5 * error_twist.cwiseProduct(error_twist).dot(error_wight_);
    if (error < eps) {
      success = true;
      break;
    }
    for (int i = 0; i < num_joints_; ++i) {
      if (joint_reach_limit[i]) body_jacobian.col(i).setZero();
    }
    Eigen::VectorXd G_x =
        body_jacobian.transpose() * (error_twist.cwiseProduct(error_wight_));
    Eigen::MatrixXd A_x =
        body_jacobian.transpose() * error_wight_.asDiagonal() * body_jacobian +
        (lamda_ * error + 1.0e-3) * kJointIdentityMat_;
    joint_pos = joint_pos + A_x.llt().solve(G_x);
    Eigen::VectorXd low_distance = joint_pos - joint_angles_limits_.col(0),
                    high_distance = joint_angles_limits_.col(1) - joint_pos;
    joint_reach_limit =
        (low_distance.cwiseMin(high_distance).array() < 0.01).array();
    joint_pos = joint_pos.cwiseMax(joint_angles_limits_.col(0))
                    .cwiseMin(joint_angles_limits_.col(1));
  }
  return success;
}

int KinematicChain::getReachLimitJointNum(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps) {
  Eigen::VectorXd low_distance = joint_angles - joint_angles_limits_.col(0),
                  high_distance = joint_angles_limits_.col(1) - joint_angles;
  Eigen::VectorXd min_error = low_distance.cwiseMin(high_distance);
  return (min_error.array() < eps).count();
}

std::vector<int> KinematicChain::getReachLimitJointId(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps) {
  std::vector<int> reach_limit_joint_id;
  Eigen::VectorXd low_distance = joint_angles - joint_angles_limits_.col(0),
                  high_distance = joint_angles_limits_.col(1) - joint_angles;
  auto judgement = (low_distance.cwiseMin(high_distance).array() < eps);
  for (int i = 0; i < judgement.size(); ++i) {
    if (judgement[i]) reach_limit_joint_id.emplace_back(i);
  }
  return reach_limit_joint_id;
}

int KinematicChain::getReachMaxLimitJointNum(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps) {
  Eigen::VectorXd high_distance = joint_angles_limits_.col(1) - joint_angles;
  return (high_distance.array() < eps).count();
}

std::vector<int> KinematicChain::getReachMaxLimitJointId(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps) {
  std::vector<int> reach_limit_joint_id;
  Eigen::VectorXd high_distance = joint_angles_limits_.col(1) - joint_angles;
  auto judgement = (high_distance.array() < eps);
  for (int i = 0; i < judgement.size(); ++i) {
    if (judgement[i]) reach_limit_joint_id.emplace_back(i);
  }
  return reach_limit_joint_id;
}

int KinematicChain::getReachMinLimitJointNum(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps) {
  Eigen::VectorXd low_distance = joint_angles - joint_angles_limits_.col(0);
  return (low_distance.array() < eps).count();
}

std::vector<int> KinematicChain::getReachMinLimitJointId(
    const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps) {
  std::vector<int> reach_limit_joint_id;
  Eigen::VectorXd low_distance = joint_angles - joint_angles_limits_.col(0);
  auto judgement = (low_distance.array() < eps);
  for (int i = 0; i < judgement.size(); ++i) {
    if (judgement[i]) reach_limit_joint_id.emplace_back(i);
  }
  return reach_limit_joint_id;
}
}  // namespace arm_controller
