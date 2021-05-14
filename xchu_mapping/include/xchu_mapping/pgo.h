//
// Created by xchu on 2021/5/12.
//

#ifndef SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_PGO_H_
#define SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_PGO_H_
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
//#include <stat.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/dataset.h> //引入头文件

//#include "scancontext/common.h"
#include "scancontext/tic_toc.h"
#include "scancontext/Scancontext.h"



typedef pcl::PointXYZI PointT;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

using namespace gtsam;
using std::cout;
using std::endl;

class PGO {
 public:
  PGO();

  void Run();

  void PerformSCLoopClosure();

  void LoopClosure();

  void ICPRefine();

  void MapVisualization();

  void SaveMap();

 private:
  ros::NodeHandle nh;
  double keyframeMeterGap;
  double movementAccumulation = 1000000.0; // large value means must add the first given frame.
  bool isNowKeyFrame = false;

  std::queue<nav_msgs::Odometry::ConstPtr> odom_queue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
  std::queue<sensor_msgs::NavSatFix::ConstPtr> gps_queue_;
  std::queue<std::pair<int, int> > loop_queue_;
  std::mutex mutex_;
  std::mutex mKF;

  std::mutex mtxICP;
  std::mutex mutex_pg_;  // when pose graph add node
  std::mutex mutex_pose_;

  std::string save_dir_;

  double curr_odom_time_ = 0;
  pcl::PointCloud<PointT>::Ptr curr_frame_;
  pcl::PointCloud<PointT>::Ptr laserCloudMapAfterPGO;
  pcl::PointCloud<PointT>::Ptr keyposes_cloud_;

  std::vector<pcl::PointCloud<PointT>::Ptr> keyframeLaserClouds;
  std::vector<std::pair<int, int> > loop_pairs_;
  std::vector<Pose6D> originPoses;
  std::vector<Pose6D> keyframePoses;
  std::vector<Pose6D> keyframePosesUpdated;
  std::vector<double> keyframeTimes;

  gtsam::NonlinearFactorGraph gtSAMgraph;
  bool gtSAMgraphMade = false;
  gtsam::Values initialEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values isamCurrentEstimate;

  Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init
  Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero

  noiseModel::Diagonal::shared_ptr priorNoise;
  noiseModel::Diagonal::shared_ptr odomNoise;
  noiseModel::Base::shared_ptr robustLoopNoise;
  noiseModel::Base::shared_ptr robustGPSNoise;

  pcl::VoxelGrid<PointT> downSizeFilterScancontext;
  SCManager scManager;
  double scDistThres;

  pcl::VoxelGrid<PointT> downSizeFilterICP;


  pcl::PointCloud<PointT>::Ptr laserCloudMapPGO;
  pcl::VoxelGrid<PointT> downSizeFilterMapPGO;
  bool laserCloudMapPGORedraw = true;

  bool useGPS = false;
  sensor_msgs::NavSatFix::ConstPtr curr_gps_;
  bool hasGPSforThisKF = false;
  bool gpsOffsetInitialized = false;
  double gpsAltitudeInitOffset = 0.0;
  double recentOptimizedX = 0.0;
  double recentOptimizedY = 0.0;

  ros::Publisher map_pub, odom_pub, final_odom_pub, pose_pub, markers_pub;
//  ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal;
  ros::Subscriber points_sub, odom_sub, gps_sub;

  void OdomCB(const nav_msgs::Odometry::ConstPtr &_laserOdometry);

  void PcCB(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes);

  void GpsCB(const sensor_msgs::NavSatFix::ConstPtr &_gps);

  void InitParams();

  Pose6D Odom2Pose6D(nav_msgs::Odometry::ConstPtr _odom);

  inline gtsam::Pose3 Pose6D2Pose3(const Pose6D &p) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
  }

  void PublishPoseAndFrame();

  void ISAM2Update();

  pcl::PointCloud<PointT>::Ptr TransformCloud2Map(const pcl::PointCloud<PointT>::Ptr &cloudIn, const Pose6D &tf);

  pcl::PointCloud<PointT>::Ptr TransformCloud2Map(pcl::PointCloud<PointT>::Ptr cloudIn, gtsam::Pose3 transformIn);

  void LoopFindNearKeyframesCloud(pcl::PointCloud<PointT>::Ptr &nearKeyframes,
                                  const int &key,
                                  const int &submap_size,
                                  const int &root_idx);

  visualization_msgs::MarkerArray CreateMarker(const ros::Time& stamp);

};

#endif //SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_PGO_H_
