#ifndef KINEMATIC_H_
#define KINEMATIC_H_

#include <Eigen/Core>
#include <random>

namespace arm_controller {

class KinematicChain {
 public:
  explicit KinematicChain(
      const std::vector<Eigen::Vector3d>& joint_axis,
      const std::vector<Eigen::Matrix3d>& joint_init_rot,
      const std::vector<Eigen::Vector3d>& joint_init_pos,
      const Eigen::Ref<const Eigen::Vector3d>& end_effector_pos_P,
      const Eigen::Ref<const Eigen::VectorXd>& joint_init_angles,
      const Eigen::Ref<const Eigen::MatrixXd>& joint_angle_limits);
  ~KinematicChain() = default;
  /**
   * @brief
   * @param joint_angles
   * @param target_pose
   */
  void forwardKinematic(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                        Eigen::Ref<Eigen::Matrix4d> target_pose);
  /**
   * @brief
   * @param joint_angles
   * @return Eigen::Matrix4d
   */
  [[nodiscard]] Eigen::Matrix4d forwardKinematic(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles);

  /**
   * @brief
   *
   * @param target_pose
   */
  void randomForwardKinematic(Eigen::Ref<Eigen::Matrix4d> target_pose);

  /**
   * @brief
   *
   * @return Eigen::Matrix4d
   */
  [[nodiscard]] Eigen::Matrix4d randomForwardKinematic();

  /**
   * @brief
   * @param joint_angles
   * @param target_pose
   * @param spatial_jacobian
   */
  void forwardKinematicWithSpatialJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
      Eigen::Ref<Eigen::Matrix4d> target_pose,
      Eigen::Ref<Eigen::MatrixXd> spatial_jacobian);
  /**
   * @brief
   * @param joint_angles
   * @param spatial_jacobian
   * @return Eigen::Matrix4d Target pose
   */
  [[nodiscard]] Eigen::Matrix4d forwardKinematicWithSpatialJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
      Eigen::Ref<Eigen::MatrixXd> spatial_jacobian);
  /**
   * @brief
   * @param joint_angles
   * @param body_jacobian
   * @return Eigen::Matrix4d
   */
  void forwardKinematicWithBodyJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
      Eigen::Ref<Eigen::Matrix4d> target_pose,
      Eigen::Ref<Eigen::MatrixXd> body_jacobian);
  /**
   * @brief
   * @param joint_angles
   * @param body_jacobian
   * @return Eigen::Matrix4d
   */
  [[nodiscard]] Eigen::Matrix4d forwardKinematicWithBodyJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
      Eigen::Ref<Eigen::MatrixXd> body_jacobian);
  /**
   * @brief
   * @param joint_angles
   * @param spatial_jacobian
   */
  void calculateSpatialJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
      Eigen::Ref<Eigen::MatrixXd> spatial_jacobian);
  /**
   * @brief
   * @param joint_angles
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] Eigen::MatrixXd calculateSpatialJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles);
  /**
   * @brief
   * @param joint_angles
   * @param body_jacobian
   */
  void calculateBodyJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
      Eigen::Ref<Eigen::MatrixXd> body_jacobian);
  /**
   * @brief
   * @param joint_angles
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] Eigen::MatrixXd calculateBodyJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles);
  /**
   * @brief
   * @param target_rot
   * @param target_pos
   * @param init_guess
   * @param joint_pos
   * @param error
   * @param max_iterations
   * @param eps
   * @return true
   * @return false
   */
  bool inverseKinematic(const Eigen::Ref<const Eigen::Matrix3d>& target_rot,
                        const Eigen::Ref<const Eigen::Vector3d>& target_pos,
                        const Eigen::Ref<const Eigen::VectorXd>& init_guess,
                        Eigen::Ref<Eigen::VectorXd> joint_pos, double& error,
                        const unsigned int max_iterations = 10,
                        const double& eps = 1.0e-3);

  /**
   * @brief
   * @param target_rot
   * @param target_pos
   * @param init_guess
   * @param joint_pos
   * @return true
   * @return false
   */
  virtual bool analyticalInvKinematic(
      const Eigen::Ref<const Eigen::Matrix3d>& target_rot,
      const Eigen::Ref<const Eigen::Vector3d>& target_pos,
      const Eigen::Ref<const Eigen::VectorXd>& init_guess,
      Eigen::Ref<Eigen::VectorXd> joint_pos) {
    throw std::runtime_error("Analytical inverse kinematic not implemented");
  }

  /**
   * @brief Set the Inverse Kinematic Solver Params object
   * @param lamda
   * @param error_wight
   */
  void setInvKinematicSolverParams(
      const double& lamda,
      const Eigen::Ref<const Eigen::VectorXd>& error_wight) {
    lamda_ = lamda;
    error_wight_ = error_wight;
  }

  /**
   * @brief Get the joint numbers that reaches the joint limit
   * @param joint_angles
   * @param eps
   */
  [[nodiscard]] int getReachLimitJointNum(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps = 0.01);

  /**
   * @brief Get the Reach Limit Joint Id object
   *
   * @param joint_angles
   * @param eps
   * @return std::vector<int>
   */
  [[nodiscard]] std::vector<int> getReachLimitJointId(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps = 0.01);
  /**
   * @brief Get the Reach Max Limit Joint Num object]
   *
   * @param joint_angles
   * @param eps
   * @return int
   */
  [[nodiscard]] int getReachMaxLimitJointNum(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps = 0.01);
  /**
   * @brief Get the Reach Max Limit Joint Id object
   *
   * @param joint_angles
   * @param eps
   * @return std::vector<int>
   */
  [[nodiscard]] std::vector<int> getReachMaxLimitJointId(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps = 0.01);
  /**
   * @brief Get the Reach Min Limit Joint Num object
   *
   * @param joint_angles
   * @param eps
   * @return int
   */
  [[nodiscard]] int getReachMinLimitJointNum(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps = 0.01);
  /**
   * @brief Get the Reach Min Limit Joint Id object
   *
   * @param joint_angles
   * @param eps
   * @return std::vector<int>
   */
  [[nodiscard]] std::vector<int> getReachMinLimitJointId(
      const Eigen::Ref<const Eigen::VectorXd>& joint_angles, double eps = 0.01);

  /**
   * @brief Set the random seed for the random forward Kinematics
   *
   * @param seed
   */
  void seed(unsigned int seed) { gen_.seed(seed); }

  /**
   * @brief Get the joint lower limit
   *
   * @return const Eigen::VectorXd
   */
  [[nodiscard]] const Eigen::VectorXd getJointLowerLimit() const {
    return joint_angles_limits_.col(0);
  }

  /**
   * @brief Get the joint upper limit
   *
   * @return const Eigen::VectorXd
   */
  [[nodiscard]] const Eigen::VectorXd getJointUpperLimit() const {
    return joint_angles_limits_.col(1);
  }

  /**
   * @brief Get the Error Weight object
   *
   * @return const Eigen::VectorXd
   */
  [[nodiscard]] const Eigen::VectorXd getErrorWeight() const {
    return error_wight_;
  }

 protected:
  // random seed for random forward kinematics
  std::mt19937 gen_{std::random_device{}()};
  // kinematic chain params
  unsigned int num_joints_;
  Eigen::VectorXd joint_init_angles_;
  Eigen::MatrixXd joint_angles_limits_;
  std::vector<Eigen::Matrix4d> screw_axis_homo_W_;
  std::vector<Eigen::Matrix4d> screw_axis_homo_B_;
  Eigen::Matrix4d init_trans_mat_, init_trans_mat_inv_;
  // inverse kinematic solver params
  Eigen::MatrixXd kJointIdentityMat_;
  Eigen::VectorXd error_wight_;
  double lamda_{0.1};
};

// class LegKinematicChain : KinematicChain {};

}  // namespace arm_controller
#endif  // GEOMETRY_KINEMATIC_H
