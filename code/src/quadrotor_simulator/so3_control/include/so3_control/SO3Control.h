#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>
#include <fstream>

class SO3Control
{
public:
  SO3Control();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d& position);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAcc(const Eigen::Vector3d& acc);
  void setOrientation(const Eigen::Quaterniond& q);
  void setOmega(const Eigen::Vector3d& omega);

  void calculateControl(const Eigen::Vector3d& des_pos,
                        const Eigen::Vector3d& des_vel,
                        const Eigen::Vector3d& des_acc, const double des_yaw,
                        const double des_yaw_dot, const Eigen::Vector3d& kx,
                        const Eigen::Vector3d& kv, const Eigen::Vector3d& kR,
                        const Eigen::Vector3d& kOm);

  const Eigen::Vector3d&    getComputedForce(void);
  const Eigen::Quaterniond& getComputedOrientation(void);
  const Eigen::Vector3d&    getComputedMoment(void);
  std::ofstream dataFile;
  std::ofstream dataFile_time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Inputs for the controller
  double          mass_;
  double          g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  Eigen::Quaterniond orientation_curr_;
  Eigen::Vector3d    omega_;
  Eigen::Matrix3d    J_;

  // Outputs of the controller
  Eigen::Vector3d    force_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d    moment_;
};

#endif
