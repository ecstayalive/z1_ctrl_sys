#include "arm_controller/geometry_utils.h"
#ifdef GEOMETRY_DEBUG
#include <iostream>
#include <stdexcept>
#endif

namespace arm_controller {

void quatToRpy(const Eigen::Vector4d &quat, Eigen::Ref<Eigen::Vector3d> rpy) {
  auto w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  rpy[0] = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  double sin_p = 2 * (w * y - x * z);
  if (std::abs(sin_p) > 1.) {
    rpy[1] = std::copysign(M_PI / 2, sin_p);
  } else {
    rpy[1] = std::asin(sin_p);
  }
  rpy[2] = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void quatToRot(const Eigen::Vector4d &quat, Eigen::Ref<Eigen::Matrix3d> rot) {
  auto w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  // quaternion to rotation matrix
  rot(0, 0) = 1 - 2 * (y * y + z * z);
  rot(0, 1) = 2 * (x * y - w * z);
  rot(0, 2) = 2 * (x * z + w * y);
  rot(1, 0) = 2 * (x * y + w * z);
  rot(1, 1) = 1 - 2 * (x * x + z * z);
  rot(1, 2) = 2 * (y * z - w * x);
  rot(2, 0) = 2 * (x * z - w * y);
  rot(2, 1) = 2 * (y * z + w * x);
  rot(2, 2) = 1 - 2 * (x * x + y * y);
}

void rpyToQuat(const Eigen::Ref<const Eigen::Vector3d> &rpy,
               Eigen::Ref<Eigen::Vector4d> quat) {
  double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
  double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  quat(0) = cr * cp * cy + sr * sp * sy;
  quat(1) = sr * cp * cy - cr * sp * sy;
  quat(2) = cr * sp * cy + sr * cp * sy;
  quat(3) = cr * cp * sy - sr * sp * cy;
}

void rpyToRot(const Eigen::Ref<const Eigen::Vector3d> &rpy,
              Eigen::Ref<Eigen::Matrix3d> rot) {
  double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
  double cr = std::cos(roll), sr = std::sin(roll);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cy = std::cos(yaw), sy = std::sin(yaw);
  rot(0, 0) = cp * cy;
  rot(0, 1) = -cr * sy + sr * sp * cy;
  rot(0, 2) = sr * sy + cr * sp * cy;
  rot(1, 0) = cp * sy;
  rot(1, 1) = cr * cy + sr * sp * sy;
  rot(1, 2) = -sr * cy + cr * sp * sy;
  rot(2, 0) = -sp;
  rot(2, 1) = sr * cp;
  rot(2, 2) = cr * cp;
}

void rotToRpy(const Eigen::Ref<const Eigen::Matrix3d> &rot,
              Eigen::Ref<Eigen::Vector3d> rpy) {
  rpy(0) = std::atan2(rot(2, 1), rot(2, 2));
  rpy(1) = std::atan2(-rot(2, 0),
                      std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
  rpy(2) = std::atan2(rot(1, 0), rot(0, 0));
}

void rotToQuat(const Eigen::Ref<const Eigen::Matrix3d> &rot,
               Eigen::Ref<Eigen::Vector4d> quat) {
  double scale;
  if (rot(2, 2) < 0) {
    if (rot(0, 0) > rot(1, 1)) {
      // use x-form
      scale = 1 + rot(0, 0) - rot(1, 1) - rot(2, 2);
      quat << rot(2, 1) - rot(1, 2), scale, rot(1, 0) + rot(0, 1),
          rot(2, 0) + rot(0, 2);
    } else {
      // use y-form
      scale = 1 - rot(0, 0) + rot(1, 1) - rot(2, 2);
      quat << rot(0, 2) - rot(2, 0), rot(1, 0) + rot(0, 1), scale,
          rot(2, 1) + rot(1, 2);
    }
  } else {
    if (rot(0, 0) < -rot(1, 1)) {
      // use z-form
      scale = 1 - rot(0, 0) - rot(1, 1) + rot(2, 2);
      quat << rot(1, 0) - rot(0, 1), rot(2, 0) + rot(0, 2),
          rot(2, 1) + rot(1, 2), scale;
    } else {
      // use w-form
      scale = 1 + rot(0, 0) + rot(1, 1) + rot(2, 2);
      quat << scale, rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0),
          rot(1, 0) - rot(0, 1);
    }
  }
  quat *= (0.5 / std::sqrt(scale));
}

void quatProduct(const Eigen::Ref<const Eigen::Vector4d> &p,
                 const Eigen::Ref<const Eigen::Vector4d> &q,
                 Eigen::Ref<Eigen::Vector4d> res) {
  double p_w = p(0), q_w = q(0);
  Eigen::Vector3d p_vector = p.tail<3>(), q_vector = q.tail<3>();
  res(0) = p_w * q_w - p_vector.dot(q_vector);
  res.tail<3>() = p_w * q_vector + q_w * p_vector + p_vector.cross(q_vector);
}

void rpyRateToAngVelBody(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                         const Eigen::Ref<const Eigen::Vector3d> &rpy_rate,
                         Eigen::Ref<Eigen::Vector3d> ang_vel_B) {
  double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]), cp = std::cos(rpy[1]),
         sp = std::cos(rpy[1]);
  Eigen::Matrix3d R;
  R << 1, 0, -sp, 0, cr, sr * cp, 0, -sr, cr * cp;
  ang_vel_B = R * rpy_rate;
}

void rpyRateToAngVelWorld(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                          const Eigen::Ref<const Eigen::Vector3d> &rpy_rate,
                          Eigen::Ref<Eigen::Vector3d> ang_vel_W) {
  double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]), cp = std::cos(rpy[1]),
         sp = std::sin(rpy[1]);
  Eigen::Matrix3d R;
  R << cp * cy, -sy, 0, cp * sy, cy, 0, -sp, 0, 1;
  ang_vel_W = R * rpy_rate;
}

void angVelWorldToRpyRate(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                          const Eigen::Ref<const Eigen::Vector3d> &ang_vel_W,
                          Eigen::Ref<Eigen::Vector3d> rpy_rate) {
  double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]), cp = std::cos(rpy[1]),
         tp = std::tan(rpy[1]);
  Eigen::Matrix3d R;
  R << cy / cp, sy / cp, 0, -sy, cy, 0, tp * cy, tp * sy, 1;
  rpy_rate = R * ang_vel_W;
}

void angVelBodyToRpyRate(const Eigen::Ref<const Eigen::Vector3d> &rpy,
                         const Eigen::Ref<const Eigen::Vector3d> &ang_vel_B,
                         Eigen::Ref<Eigen::Vector3d> rpy_rate) {
  double sr = std::sin(rpy[0]), cr = std::cos(rpy[0]), cp = std::cos(rpy[1]),
         tp = std::tan(rpy[1]);
  Eigen::Matrix3d R;
  R << 1, sr * tp, cr * tp, 0, cr, -sr, 0, sr / cp, cr / cp;
  rpy_rate = R * ang_vel_B;
}

void rotateVecByQuat(const Eigen::Ref<const Eigen::Vector3d> &v,
                     const Eigen::Ref<const Eigen::Vector4d> &q,
                     Eigen::Ref<Eigen::Vector3d> res) {
  Eigen::Vector3d u = q.segment<3>(1);  // x y z
  double w = q[0];
  res = 2.0f * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.0f * w * u.cross(v);
}

void rotateVecByInvQuat(const Eigen::Ref<const Eigen::Vector3d> &v,
                        const Eigen::Ref<const Eigen::Vector4d> &q,
                        Eigen::Ref<Eigen::Vector3d> res) {
  Eigen::Vector3d u = -q.segment<3>(1);  // x y z
  double w = q[0];
  res = 2.0f * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.0f * w * u.cross(v);
}

void so3ToRot(const Eigen::Ref<const Eigen::Matrix3d> &skew_mat,
              const double &theta, Eigen::Ref<Eigen::Matrix3d> rot) {
  rot = Eigen::Matrix3d::Identity() + std::sin(theta) * skew_mat +
        (1 - std::cos(theta)) * skew_mat * skew_mat;
}

void angleAxisToRot(const Eigen::Ref<const Eigen::Vector3d> &axis_vec,
                    const double &theta, Eigen::Ref<Eigen::Matrix3d> rot) {
  Eigen::Matrix3d skew_mat;
  vecToSkewMat(axis_vec, skew_mat);
  rot = Eigen::Matrix3d::Identity() + std::sin(theta) * skew_mat +
        (1 - std::cos(theta)) * skew_mat * skew_mat;
}

void rotToAngleAxis(const Eigen::Ref<const Eigen::Matrix3d> &rot,
                    Eigen::Ref<Eigen::Vector3d> axis_vec, double &theta) {
  // theta belong to [0, pi]
  double c_theta = std::max(std::min(0.5 * (rot.trace() - 1), 1.0), -1.0);
  theta = std::acos(c_theta);
  if (theta < 1.0e-6) {
    // The rotation matrix is I, axis_vec is actually undefined
    // but for the sake of formal unity, assign it a unit vector
    axis_vec << 0., 0., 1.;
  } else {
    double scale;
    unsigned int max_row_id{0};
    if (rot(1, 1) > rot(max_row_id, max_row_id)) max_row_id = 1;
    if (rot(2, 2) > rot(max_row_id, max_row_id)) max_row_id = 2;
    switch (max_row_id) {
      case 0:
        scale = 1 + rot(0, 0) - rot(1, 1) - rot(2, 2);
        axis_vec << scale, rot(1, 0) + rot(0, 1), rot(2, 0) + rot(0, 2);
        break;
      case 1:
        scale = 1 - rot(0, 0) + rot(1, 1) - rot(2, 2);
        axis_vec << rot(1, 0) + rot(0, 1), scale, rot(2, 1) + rot(1, 2);
        break;
      case 2:
        scale = 1 - rot(0, 0) - rot(1, 1) + rot(2, 2);
        axis_vec << rot(2, 0) + rot(0, 2), rot(2, 1) + rot(1, 2), scale;
        break;
      default:
        throw std::runtime_error("rotToAngleAxis function runtime error!!!");
    }
    axis_vec *= (1 / std::sqrt(2 * (1 - c_theta) * scale));
  }
}

void twistToTwistHomo(const Eigen::Ref<const Eigen::VectorXd> &twist,
                      Eigen::Ref<Eigen::Matrix4d> twist_homo) {
  Eigen::Vector3d angular_vel = twist.head(3), linear_vel = twist.tail(3);
  twist_homo.setZero();
  vecToSkewMat(angular_vel, twist_homo.block<3, 3>(0, 0));
  twist_homo.block<3, 1>(0, 3) = linear_vel;
}
Eigen::Matrix4d twistToTwistHomo(
    const Eigen::Ref<const Eigen::VectorXd> &twist) {
  Eigen::Matrix4d twist_homo = Eigen::Matrix4d::Zero();
  Eigen::Vector3d angular_vel = twist.head(3), linear_vel = twist.tail(3);
  vecToSkewMat(angular_vel, twist_homo.block<3, 3>(0, 0));
  twist_homo.block<3, 1>(0, 3) = linear_vel;
  return twist_homo;
}

void twistHomoToTwist(const Eigen::Ref<const Eigen::Matrix4d> &twist_homo,
                      Eigen::Ref<Eigen::VectorXd> twist) {
  Eigen::Matrix3d angular_skew_mat = twist_homo.block<3, 3>(0, 0);
  Eigen::Vector3d linear_vel = twist_homo.block<3, 1>(0, 3);
  skewMatToVec(angular_skew_mat, twist.head<3>());
  twist.tail<3>() = linear_vel;
}
Eigen::VectorXd twistHomoToTwist(
    const Eigen::Ref<const Eigen::Matrix4d> &twist_homo) {
  Eigen::VectorXd twist(6);
  Eigen::Matrix3d angular_skew_mat = twist_homo.block<3, 3>(0, 0);
  Eigen::Vector3d linear_vel = twist_homo.block<3, 1>(0, 3);
  skewMatToVec(angular_skew_mat, twist.head<3>());
  twist.tail<3>() = linear_vel;
  return twist;
}

void adjointMat(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                Eigen::Ref<Eigen::MatrixXd> adjoint_mat) {
  Eigen::Matrix3d rot;
  Eigen::Vector3d pos;
  transMatToRotAndPos(trans_mat, rot, pos);
  adjoint_mat.setZero();
  adjoint_mat.block<3, 3>(0, 0) = rot;
  adjoint_mat.block<3, 3>(3, 0) = vecToSkewMat(pos) * rot;
  adjoint_mat.block<3, 3>(3, 3) = rot;
}

Eigen::MatrixXd adjointMat(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat) {
  Eigen::MatrixXd adjoint_mat(6, 6);
  Eigen::Matrix3d rot;
  Eigen::Vector3d pos;
  transMatToRotAndPos(trans_mat, rot, pos);
  adjoint_mat.setZero();
  adjoint_mat.block<3, 3>(0, 0) = rot;
  adjoint_mat.block<3, 3>(3, 0) = vecToSkewMat(pos) * rot;
  adjoint_mat.block<3, 3>(3, 3) = rot;
  return adjoint_mat;
}

void adjointMapToTwistHomo(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                           const Eigen::Ref<const Eigen::VectorXd> &twist,
                           Eigen::Ref<Eigen::Matrix4d> result_twist_homo) {
  // result = adjointMat(trans_mat) * twist;
  Eigen::Matrix4d trans_mat_inv = Eigen::Matrix4d::Identity();
  trans_mat_inv.block<3, 3>(0, 0) = trans_mat.block<3, 3>(0, 0).transpose();
  trans_mat_inv.block<3, 1>(0, 3) =
      -trans_mat_inv.block<3, 3>(0, 0) * trans_mat.block<3, 1>(0, 3);
  Eigen::Matrix4d twist_homo;
  twistToTwistHomo(twist, twist_homo);
  result_twist_homo = trans_mat * twist_homo * trans_mat_inv;
}
Eigen::Matrix4d adjointMapToTwistHomo(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    const Eigen::Ref<const Eigen::VectorXd> &twist) {
  Eigen::Matrix4d result_twist_homo;
  adjointMapToTwistHomo(trans_mat, twist, result_twist_homo);
  return result_twist_homo;
}

void adjointMapTwistHomoToTwistHomo(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    const Eigen::Ref<const Eigen::Matrix4d> &twist_homo,
    Eigen::Ref<Eigen::Matrix4d> result_twist_homo) {
  // result = adjointMat(trans_mat) * twist;
  Eigen::Matrix4d trans_mat_inv = Eigen::Matrix4d::Identity();
  trans_mat_inv.block<3, 3>(0, 0) = trans_mat.block<3, 3>(0, 0).transpose();
  trans_mat_inv.block<3, 1>(0, 3) =
      -trans_mat_inv.block<3, 3>(0, 0) * trans_mat.block<3, 1>(0, 3);
  result_twist_homo = trans_mat * twist_homo * trans_mat_inv;
}
Eigen::Matrix4d adjointMapTwistHomoToTwistHomo(
    const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
    const Eigen::Ref<const Eigen::Matrix4d> &twist) {
  Eigen::Matrix4d result_twist_homo;
  adjointMapTwistHomoToTwistHomo(trans_mat, twist, result_twist_homo);
  return result_twist_homo;
}

void adjointMap(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                const Eigen::Ref<const Eigen::VectorXd> &twist,
                Eigen::Ref<Eigen::VectorXd> result) {
  // result = adjointMat(trans_mat) * twist;
  Eigen::Matrix4d trans_mat_inv = Eigen::Matrix4d::Identity();
  trans_mat_inv.block<3, 3>(0, 0) = trans_mat.block<3, 3>(0, 0).transpose();
  trans_mat_inv.block<3, 1>(0, 3) =
      -trans_mat_inv.block<3, 3>(0, 0) * trans_mat.block<3, 1>(0, 3);
  Eigen::Matrix4d twist_homo;
  twistToTwistHomo(twist, twist_homo);
  Eigen::Matrix4d result_homo = trans_mat * twist_homo * trans_mat_inv;
  twistHomoToTwist(result_homo, result);
}
Eigen::VectorXd adjointMap(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                           const Eigen::Ref<const Eigen::VectorXd> &twist) {
  Eigen::VectorXd result(6);
  adjointMap(trans_mat, twist, result);
  return result;
}

void twistToScrew(const Eigen::Ref<const Eigen::VectorXd> &twist,
                  Eigen::Ref<Eigen::VectorXd> screw_axis, double &d_theta) {
  Eigen::Vector3d angular_vel = twist.head(3), linear_vel = twist.head(3);
  if (std::abs(angular_vel.norm()) < 1.0e-6) {
    d_theta = linear_vel.norm();
  } else {
    d_theta = angular_vel.norm();
  }
  screw_axis = twist / d_theta;
}

void se3ToTransMat(const Eigen::Ref<const Eigen::Matrix4d> &screw_skew_mat,
                   const double &theta, Eigen::Ref<Eigen::Matrix4d> trans_mat) {
  Eigen::Matrix3d angular_skew_mat = screw_skew_mat.block<3, 3>(0, 0);
  Eigen::Vector3d vel_axis = screw_skew_mat.block<3, 1>(0, 3);
  trans_mat.setIdentity();
  so3ToRot(angular_skew_mat, theta, trans_mat.block<3, 3>(0, 0));
  trans_mat.block<3, 1>(0, 3) =
      (theta * Eigen::Matrix3d::Identity() +
       (1 - std::cos(theta)) * angular_skew_mat +
       (theta - std::sin(theta)) * angular_skew_mat * angular_skew_mat) *
      vel_axis;
}

Eigen::Matrix4d se3ToTransMat(
    const Eigen::Ref<const Eigen::Matrix4d> &screw_skew_mat,
    const double &theta) {
  Eigen::Matrix4d trans_mat;
  se3ToTransMat(screw_skew_mat, theta, trans_mat);
  return trans_mat;
}

void screwToTransMat(const Eigen::Ref<const Eigen::VectorXd> &screw_axis,
                     const double &theta,
                     Eigen::Ref<Eigen::Matrix4d> trans_mat) {
  Eigen::Vector3d angular_axis = screw_axis.head(3),
                  vel_axis = screw_axis.tail(3);
  Eigen::Matrix3d angular_skew_mat = vecToSkewMat(angular_axis);
  trans_mat.setIdentity();
  angleAxisToRot(angular_axis, theta, trans_mat.block<3, 3>(0, 0));
  trans_mat.block<3, 1>(0, 3) =
      (theta * Eigen::Matrix3d::Identity() +
       (1 - std::cos(theta)) * angular_skew_mat +
       (theta - std::sin(theta)) * angular_skew_mat * angular_skew_mat) *
      vel_axis;
}
Eigen::Matrix4d screwToTransMat(
    const Eigen::Ref<const Eigen::VectorXd> &screw_axis, const double &theta) {
  Eigen::Matrix4d homo_mat;
  screwToTransMat(screw_axis, theta, homo_mat);
  return homo_mat;
}

void twistToTransMat(const Eigen::Ref<const Eigen::VectorXd> &twist,
                     Eigen::Ref<Eigen::Matrix4d> trans_mat) {
  double theta;
  Eigen::VectorXd screw_axis(6);
  twistToScrew(twist, screw_axis, theta);
  screwToTransMat(screw_axis, theta, trans_mat);
}

void transMatToScrew(const Eigen::Ref<const Eigen::Matrix4d> &trans_mat,
                     Eigen::Ref<Eigen::VectorXd> screw_axis, double &theta) {
  Eigen::Vector3d angular_axis;
  double angular_theta;
  rotToAngleAxis(trans_mat.block<3, 3>(0, 0), angular_axis, angular_theta);
  if (angular_theta < 1.0e-6) {
    screw_axis.head<3>().setZero();
    theta = trans_mat.block<3, 1>(0, 3).norm();
    screw_axis.tail<3>() =
        trans_mat.block<3, 1>(0, 3) / std::max(theta, 1.0e-6);
  } else {
    screw_axis.head<3>() = angular_axis;
    theta = angular_theta;
    Eigen::Matrix3d G_inv, angular_skew_mat;
    vecToSkewMat(angular_axis, angular_skew_mat);
    G_inv = Eigen::Matrix3d::Identity() / theta - 0.5 * angular_skew_mat +
            (1 / theta - 0.5 / std::tan(theta / 2)) * angular_skew_mat *
                angular_skew_mat;
    screw_axis.tail<3>() = G_inv * trans_mat.block<3, 1>(0, 3);
  }
}

void posToLpy(const Eigen::Ref<const Eigen::Vector3d> &pos,
              Eigen::Ref<Eigen::Vector3d> lpy) {
  lpy(0) = pos.norm();
  lpy(1) = std::atan2(pos(2), pos.head(2).norm());
  lpy(2) = std::atan2(pos(1), pos(0));
}

void lpyToPos(const Eigen::Ref<const Eigen::Vector3d> &lpy,
              Eigen::Ref<Eigen::Vector3d> pos) {
  pos(0) = lpy(0) * std::cos(lpy(1)) * std::cos(lpy(2));
  pos(1) = lpy(0) * std::cos(lpy(1)) * std::sin(lpy(2));
  pos(2) = lpy(0) * std::sin(lpy(1));
}

void lpyRateToLinVel(const Eigen::Ref<const Eigen::Vector3d> &lpy,
                     const Eigen::Ref<const Eigen::Vector3d> &lpy_rate,
                     Eigen::Ref<Eigen::Vector3d> lin_vel_W) {
  double l = lpy[0], cp = std::cos(lpy[1]), sp = std::sin(lpy[1]),
         cy = std::cos(lpy[2]), sy = std::sin(lpy[2]);
  Eigen::Matrix3d R;
  R << cp * cy, -l * sp * cy, -l * cp * sy, cp * sy, -l * sp * sy, l * cp * cy,
      sp, l * cp, 0;
  lin_vel_W = R * lpy_rate;
}

void linVelToLpyRate(const Eigen::Ref<const Eigen::Vector3d> &lpy,
                     const Eigen::Ref<const Eigen::Vector3d> &lin_vel,
                     Eigen::Ref<Eigen::Vector3d> lpy_rate) {
  double l = lpy[0], cp = std::cos(lpy[1]), sp = std::sin(lpy[1]),
         cy = std::cos(lpy[2]), sy = std::sin(lpy[2]);
  Eigen::Matrix3d R;
  R << cp * cy, sy * cp, sp, -sp * cy / l, -sp * sy / l, cp / l, -sy / (l * cp),
      cy / (l * cp), 0;
  lpy_rate = R * lin_vel;
}

}  // namespace arm_controller
