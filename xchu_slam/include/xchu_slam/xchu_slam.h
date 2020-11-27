/**
 * @file xchu_slam.h
 * @author xchu
 * 
 */

#ifndef __XCHUSlam__
#define __XCHUSlam__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <array>
#include <thread>
#include <memory>
#include <pthread.h>
#include <chrono>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <xchu_slam/ground_filter.hpp>

#include <pclomp/ndt_omp.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

using namespace gtsam;

struct POSE {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  Eigen::Matrix4f t;

  POSE() {
    init();
  }

  void init() {
    x = y = z = 0.0;
    roll = pitch = yaw = 0.0;
    t.setIdentity();
  }

  void updateT() {
    t.setIdentity();
    Eigen::Translation3d tf_trans(x, y, z);
    Eigen::AngleAxisd rot_x(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(yaw, Eigen::Vector3d::UnitZ());
    t = (tf_trans * rot_z * rot_y * rot_x).matrix().cast<float>();
  }
};

struct PointXYZIRPYT {
  PCL_ADD_POINT4D

  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll,
                                                                                                       roll)(float,
                                                                                                             pitch,
                                                                                                             pitch)(
                                      float, yaw, yaw)(double, time, time))

using PointTPose = PointXYZIRPYT;
using PointT = pcl::PointXYZI;

static double max_localmap_size = 30.0, odom_size = 0.0, localmap_size = 0.0;

class XCHUSlam {
 public:
  XCHUSlam(ros::NodeHandle nh, ros::NodeHandle pnh);

  ~XCHUSlam();

  /**
   * 可视化线程
   */
  void visualThread();

  /**
   * 回环检测线程
   */
  void loopClosureThread();

  /**
   * 保存全局地图
   */
  void transformAndSaveFinalMap();

 private:

  bool init();

  /**
   * 编码器数据处理
   * @param msg
   */
  void odomCB(const nav_msgs::OdometryConstPtr &msg);

  /**
   * 点云数据callback
   */
  void pcCB(const sensor_msgs::PointCloud2ConstPtr &msg);

  void pcCB2(const sensor_msgs::PointCloud2ConstPtr &msg);

  /**
   * IMU数据处理
   */
  void imuCB(const sensor_msgs::ImuConstPtr &msg);

  /**
   * GPS数据处理
   * @param msg
   */
  void gpsCB(const nav_msgs::OdometryConstPtr &msg);

  /**
   * 处理点云数据
   */
  void processCloud(const pcl::PointCloud<PointT> &tmp, const ros::Time &scan_time);

  /**
   * 点云去畸变,参考loam,基于匀速运动假设
   */
  void adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time);

  /**
   * 提取和更新局部地图
   * 使用autoware cpu_ndt时可以通过updateVoxelGrid更新target map
   */
  void extractLocalmap();

  void updateLocalmap();

  /**
   * 保存关键帧和更新因子图
   * 移动距离作为关键帧选取标准
   */
  bool saveKeyframesAndFactor();

  /**
   * 可视化线程中发布关键帧和全局地图
   */
  void publishKeyposesAndFrames();

  /**
   * @brief 回环检测及位姿图更新
   * 利用欧氏距离、里程距离、时间作为筛选条件，找到最近的关键帧，并拼接前后的N帧作为localmap
   * 最后ICP匹配添加回环约束
   */
  void performLoopClosure();

  /**
   * @brief 基于里程计的回环检测，
   * 直接提取当前位姿附近(radiusSearch)的 keyframe作为icp的target
   */
  bool detectLoopClosure();

  /**
   * 检测到回环后执行回环优化修正位姿及target map
   */
  void correctPoses();

  void imuOdomCalc(ros::Time current_time);

  void imuCalc(ros::Time current_time);

  void odomCalc(ros::Time current_time);

  void imuUpSideDown(sensor_msgs::Imu &input);

  void addGPSFactor();

  float pointDistance(PointT p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  }

  float pointDistance(PointT p1, PointT p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
  }

  inline double warpToPm(double a_num, const double a_max) {
    if (a_num >= a_max) {
      a_num -= 2.0 * a_max;
    }
    return a_num;
  }

  inline double warpToPmPi(double a_angle_rad) {
    return warpToPm(a_angle_rad, M_PI);
  }

  inline double calcDiffForRadian(const double lhs_rad, const double rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
      diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
  }

  void setGeometryOrient(geometry_msgs::PoseWithCovarianceStamped &msg, float w, float x, float y, float z) {
    msg.pose.pose.orientation.w = w;
    msg.pose.pose.orientation.x = x;
    msg.pose.pose.orientation.y = y;
    msg.pose.pose.orientation.z = z;
  }

  inline void setGeometryPosition(geometry_msgs::PoseWithCovarianceStamped &msg, float x, float y, float z) {
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
  }

  inline pcl::PointCloud<PointT>::Ptr
  transformPointCloud(const pcl::PointCloud<PointT>::ConstPtr cloud_in, const PointTPose &trans) {
    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
    this_transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()))
        .toRotationMatrix();
    this_transformation(0, 3) = trans.x;
    this_transformation(1, 3) = trans.y;
    this_transformation(2, 3) = trans.z;
    pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*cloud_in, *tf_cloud, this_transformation);
    return tf_cloud;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_pc_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_gps_;

  ros::Publisher pub_keyposes_;
  ros::Publisher pub_globalmap_;

  ros::Publisher pub_undistorted_pc_;
  ros::Publisher pub_predict_odom_;
  ros::Publisher pub_final_odom_;

  ros::Publisher pub_history_keyframes_;
  ros::Publisher pub_icp_keyframes_;
  ros::Publisher pub_localmap_, pub_current_frames_;

  utils::RayGroundFilter ground_filter;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::Transform tf_m2o_;
  tf::Transform tf_o2b_;

  bool imu_upside_down_ = false;

  // gtsam 相关
  NonlinearFactorGraph gtSAMgraph_;
  Values initial_estimate_;
  ISAM2 *isam;
  Values isam_current_estimate_;
  Eigen::MatrixXd poseCovariance;

  noiseModel::Diagonal::shared_ptr prior_noise_;
  noiseModel::Diagonal::shared_ptr odom_noise_;
  noiseModel::Diagonal::shared_ptr constraint_noise_;

  // GPS Settings
  bool useImuHeadingInitialization;
  bool useGpsElevation = false;
  float gpsCovThreshold;
  float poseCovThreshold;

  std::deque<nav_msgs::Odometry> gps_deque_;
  bool system_initialized_ = false;

  // imu 相关
  static const int imu_queue_len_ = 100;

  int imu_ptr_front_, imu_ptr_last_, imu_ptr_last_iter_;
  Eigen::Vector3f rpy_cur_, velo_xyz_cur_, shift_xyz_cur_;
  Eigen::Vector3f rpy_start_, velo_xyz_start_, shift_xyz_start_;
  Eigen::Vector3f shift_from_start_;

  std::array<double, imu_queue_len_> imu_time_;
  std::array<float, imu_queue_len_> imu_roll_;
  std::array<float, imu_queue_len_> imu_pitch_;
  std::array<float, imu_queue_len_> imu_yaw_;

  std::array<float, imu_queue_len_> imu_acc_x_;
  std::array<float, imu_queue_len_> imu_acc_y_;
  std::array<float, imu_queue_len_> imu_acc_z_;
  std::array<float, imu_queue_len_> imu_velo_x_;
  std::array<float, imu_queue_len_> imu_velo_y_;
  std::array<float, imu_queue_len_> imu_velo_z_;
  std::array<float, imu_queue_len_> imu_shift_x_;
  std::array<float, imu_queue_len_> imu_shift_y_;
  std::array<float, imu_queue_len_> imu_shift_z_;

  std::array<float, imu_queue_len_> imu_angular_velo_x_;
  std::array<float, imu_queue_len_> imu_angular_velo_y_;
  std::array<float, imu_queue_len_> imu_angular_velo_z_;
  std::array<float, imu_queue_len_> imu_angular_rot_x_;
  std::array<float, imu_queue_len_> imu_angular_rot_y_;
  std::array<float, imu_queue_len_> imu_angular_rot_z_;

  // 里程计相关
//  int odom_ptr_front_, odom_ptr_last_, odom_ptr_last_iter_;
//  std::array<nav_msgs::Odometry, imu_queue_len_> odom_queue_;
//  nav_msgs::Odometry pre_odom_, cur_odom_;


  std::queue<sensor_msgs::PointCloud2> cloudBuf;
  std::mutex mutex_lock;

  std::string _imu_topic;  // 定义imu消息的topic
  std::string _odom_topic;
  std::string _lidar_topic;
  std::string _gps_topic;

  // pose相关
  POSE previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom, diff_pose, diff_imu_pose,
      diff_odom_pose, diff_imu_odom_pose;
  POSE current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom, init_pose;
  POSE added_pose;  //  added_pose记录点云加入地图时候的位置         // 初始设为0即可,因为第一帧如论如何也要加入地图的

  Eigen::Matrix4f pre_pose_, guess_pose_, guess_pose_imu_, guess_pose_odom_, guess_pose_imu_odom_;
  Eigen::Matrix4f current_pose_, current_pose_imu_, current_pose_odom_, current_pose_imu_odom_;
  Eigen::Matrix4f ndt_pose_, localizer_pose_, added_pose_;

  ros::Time current_scan_time;
  ros::Time previous_scan_time;
  ros::Duration scan_duration;

  // 设置变量,用以接受imu和odom消息
  sensor_msgs::Imu imu;
  nav_msgs::Odometry odom;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  // 定义速度值 --包括实际速度值,和imu取到的速度值
  double current_velocity_x;
  double current_velocity_y;
  double current_velocity_z;

  double current_velocity_imu_x;
  double current_velocity_imu_y;
  double current_velocity_imu_z;

  // ndt 相关
  geometry_msgs::PoseWithCovarianceStamped pre_pose_ndt_, cur_pose_ndt_;
  geometry_msgs::PoseWithCovarianceStamped pre_keypose_;
  geometry_msgs::PoseWithCovarianceStamped predict_pose_;
  Eigen::Matrix4f pre_pose_m_, cur_pose_m_, pre_pose_o_, cur_pose_o_, diff_pose_;
  Eigen::Matrix4f t_base_link;
  Eigen::Matrix4f final_transformation_;
  double fitness_score_;
  bool has_converged_;
  int final_iters_;

  cpu::NormalDistributionsTransform<PointT, PointT> cpu_ndt_;
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp_;

  pcl::PointCloud<PointT>::Ptr pc_source_;
  pcl::PointCloud<PointT>::Ptr pc_target_;
  pcl::PointCloud<PointT>::Ptr localmap_ptr;

  pcl::PointCloud<PointT>::Ptr cloud_keyposes_3d_;
  pcl::PointCloud<PointTPose>::Ptr cloud_keyposes_6d_;

  std::vector<pcl::PointCloud<PointT>::Ptr> cloud_keyframes_;

  pcl::KdTreeFLANN<PointT>::Ptr kdtree_poses_;



  // 回环检测相关
  bool loop_closed_ = false;
  int latest_history_frame_id_;
  int closest_history_frame_id_;
  pcl::PointCloud<PointT>::Ptr latest_keyframe_;
  pcl::PointCloud<PointT>::Ptr near_history_keyframes_;
  std::deque<pcl::PointCloud<PointT>::Ptr> recent_keyframes_;
  std::mutex mtx_;

  // 地图相关
  pcl::PointCloud<PointT> localmap, submap;  // 此处定义地图  --global map
  double min_add_scan_shift = 0.5;  // 点云添加到locaMap里的最小间隔值
  double distance_shift = 0.0;
  // 参数相关
  float scan_period_;
  bool use_odom_, use_imu_, use_gps_;
  float keyframe_dist_; // 移动距离作为关键帧提取参考
  Eigen::Matrix4f tf_b2l_ = Eigen::Matrix4f::Identity();
  int surround_search_num_;      // 提取附近点云
  float surround_search_radius_; // kdtree 搜索参数
  float voxel_leaf_size_;        // 对地图点云进行降采样
  float min_scan_range_, max_scan_range_;

  double trans_eps_, step_size, ndt_res_; // ndt 配准参数
  int max_iters_;

  float history_search_radius_; // 回环检测参数
  int history_search_num_;
  float history_fitness_score_;
  float ds_history_size_;

  std::string save_dir_;

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterGlobalMap; // for global map visualization
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterGlobalMapKeyFrames; // for global map visualization
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterLocalmap; // for global map visualization
  pcl::VoxelGrid<PointT> downSizeFilterSource;
  pcl::VoxelGrid<PointT> downSizeFilterHistoryKeyframes;
};

#endif