#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>
#include<queue>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include "input.h"

class SO3Control
{
public:
  SO3Control(Parameter_t param);

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d &position);
  void setVelocity(const Eigen::Vector3d &velocity);
  void setAcc(const Eigen::Vector3d &acc);
  void setQuat(const Eigen::Quaterniond &q);

  quadrotor_msgs::Px4ctrlDebug calculateControl(const Eigen::Vector3d &des_pos,
                        const Eigen::Vector3d &des_vel,
                        const Eigen::Vector3d &des_acc,
                        const Eigen::Vector3d &des_jer,
                        const Eigen::Vector3d &des_dir,
                        const Eigen::Vector3d &des_dir_dot,
                        const double des_yaw, const double des_yaw_dot,
                        const Imu_Data_t &imu,const Odom_Data_t &odom);

  bool forward(const Eigen::Vector3d &vel,
               const Eigen::Vector3d &acc,
               const Eigen::Vector3d &jer,
               const Eigen::Vector3d &dir,
               const Eigen::Vector3d &ddir,
               double &psi,
               double &dpsi,
               double &thr,
               Eigen::Vector4d &quat,
               Eigen::Vector3d &omg);

  const Eigen::Vector3d &getComputedForce(void);
  const double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  void resetThrustMapping(void);
  const Eigen::Quaterniond &getComputedOrientation(void);
  const Eigen::Vector3d &getComputedOmega(void);
  const double &getComputedYaw(void);
  const double &getComputedYawDot(void);
  const bool estimateThrustModel(const Eigen::Vector3d &est_a,const Parameter_t &param);


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  // Inputs for the controller
  double mass_;
  double g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  Eigen::Quaterniond quat_;
  Eigen::Vector3d old_omega_;

  // Outputs of the controller
  double yaw_, yaw_dot_;
  Eigen::Vector3d force_;
  Eigen::Vector3d omega_;
  Eigen::Quaterniond orientation_;

  // thrust model
  double thr2acc_;
  const double rho2_ = 0.998; // do not change
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  double P_;
};

#endif
