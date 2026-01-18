#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

static const double g = 9.81;
static const double T_END = 2 * M_PI;
static const double DT = 0.02;
static const double DIFF_EPS = 1e-3;  // 用于数值微分

// 轨迹位置
Eigen::Vector3d pos(double t) {
  double s = std::sin(t), c = std::cos(t);
  double denom = 1.0 + s * s;
  double x = 10.0 * c / denom;
  double y = 10.0 * s * c / denom;
  double z = 10.0;
  return Eigen::Vector3d(x, y, z);
}

// 数值微分
Eigen::Vector3d vel(double t) {
  double tp = std::min(t + DIFF_EPS, T_END);
  double tm = std::max(t - DIFF_EPS, 0.0);
  return (pos(tp) - pos(tm)) / (tp - tm);
}

Eigen::Vector3d acc(double t) {
  double tp = std::min(t + DIFF_EPS, T_END);
  double tm = std::max(t - DIFF_EPS, 0.0);
  return (vel(tp) - vel(tm)) / (tp - tm);
}

// 旋转矩阵转四元数 (x,y,z,w)，确保归一化
Eigen::Vector4d rotToQuat(const Eigen::Matrix3d &R) {
  Eigen::Vector4d q;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = std::sqrt(tr + 1.0) * 2.0;
    q[3] = 0.25 * S;
    q[0] = (R(2, 1) - R(1, 2)) / S;
    q[1] = (R(0, 2) - R(2, 0)) / S;
    q[2] = (R(1, 0) - R(0, 1)) / S;
  } else {
    int i = (R(0, 0) > R(1, 1))
                ? ((R(0, 0) > R(2, 2)) ? 0 : 2)
                : ((R(1, 1) > R(2, 2)) ? 1 : 2);
    int j = (i + 1) % 3, k = (i + 2) % 3;
    double S = std::sqrt(R(i, i) - R(j, j) - R(k, k) + 1.0) * 2.0;
    q[i] = 0.25 * S;
    q[3] = (R(k, j) - R(j, k)) / S;
    q[j] = (R(j, i) + R(i, j)) / S;
    q[k] = (R(k, i) + R(i, k)) / S;
  }
  q.normalize();
  return q;
}

int main() {
  // 确保 solutions 目录存在
  std::string out_dir = "/home/stuwork/MRPC-2025-homework/MRPC-2025-homework/solutions";
  mkdir(out_dir.c_str(), 0755);

  std::ofstream fout(out_dir + "/df_quaternion.csv");
  if (!fout.is_open()) {
    std::cerr << "Failed to open output file\n";
    return 1;
  }
  fout << "t,qx,qy,qz,qw\n";

  Eigen::Vector4d q_prev = Eigen::Vector4d::Zero();

  for (double t = 0.0; t <= T_END + 1e-9; t += DT) {
    Eigen::Vector3d p = pos(t);
    Eigen::Vector3d v = vel(t);
    Eigen::Vector3d a = acc(t);

    double psi = std::atan2(v.y(), v.x());
    Eigen::Vector3d bx(std::cos(psi), std::sin(psi), 0.0);

    Eigen::Vector3d f = a + g * Eigen::Vector3d(0, 0, 1);  // 推力方向
    Eigen::Vector3d bz = f.normalized();
    Eigen::Vector3d by = bz.cross(bx).normalized();
    bx = by.cross(bz).normalized();  // 重新正交化

    Eigen::Matrix3d R;
    R.col(0) = bx;
    R.col(1) = by;
    R.col(2) = bz;

    Eigen::Vector4d q = rotToQuat(R);  // (x,y,z,w)

    // 连续性：与上一帧点积 < 0 则取反
    if (q_prev.norm() > 0 && q_prev.dot(q) < 0) q = -q;
    // 确保 qw >= 0
    if (q[3] < 0) q = -q;

    q_prev = q;

    fout << std::fixed << std::setprecision(2) << t << ","
         << std::setprecision(7) << q[0] << "," << q[1] << "," << q[2] << ","
         << q[3] << "\n";
  }

  fout.close();
  std::cout << "Saved to " << out_dir << "/df_quaternion.csv\n";
  return 0;
}
