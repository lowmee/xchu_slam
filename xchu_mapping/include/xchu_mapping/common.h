//
// Created by xchu on 2021/5/16.
//

#ifndef SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_COMMON_H_
#define SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_COMMON_H_

typedef pcl::PointXYZI PointT;

inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Pose6D() {
    x = y = z = roll = pitch = yaw = 0.0;
  };

  Pose6D(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
      : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {
  };
};

extern Pose6D operator+(const Pose6D &A, const Pose6D B) //给结构体定义加法；
{
  return Pose6D{A.x + B.x, A.y + B.y, A.z + B.z, A.roll + B.roll, A.pitch + B.pitch, A.yaw + B.yaw};
}
extern Pose6D operator-(const Pose6D &A, const Pose6D B) {
  return Pose6D{A.x - B.x, A.y - B.y, A.z - B.z, A.roll - B.roll, A.pitch - B.pitch, A.yaw - B.yaw};
}
extern std::ostream &operator<<(std::ostream &out, const Pose6D &p)  //定义结构体流输出
{
  out << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << "," << p.yaw << ")";
  return out;
}

extern Pose6D Matrix2Pose6D(Eigen::Matrix4d matrix) {
  Eigen::Vector3d pos = matrix.block<3, 1>(0, 3).matrix();
  //  Eigen::Vector3d pos = t_base_link.block<3, 1>(0, 3).matrix();
  Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0).matrix();
  Eigen::Quaterniond quat(rot);

  Pose6D p;
  p.x = pos(0);
  p.y = pos(1);
  p.z = pos(2);
  tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(p.roll, p.pitch, p.yaw);
  return p;
}
extern Eigen::Matrix4d Pose6D2Matrix(Pose6D p) {
  Eigen::Translation3d tf_trans(p.x, p.y, p.z);
  Eigen::AngleAxisd rot_x(p.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_y(p.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot_z(p.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();
  return mat;
}

#endif //SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_COMMON_H_