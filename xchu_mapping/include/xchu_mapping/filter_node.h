//
// Created by xchu on 2021/4/13.
//

#ifndef SRC_XCHU_MAPPING_SRC_POINTS_FILTER_NODE_H_
#define SRC_XCHU_MAPPING_SRC_POINTS_FILTER_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <thread>
#include <mutex>
#include <queue>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp> /*找不到*/
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "xchu_mapping/FloorCoeffs.h"

using PointT = pcl::PointXYZI;

class CloudFilter {
 public:
  CloudFilter();

  void Run();

 private:
  ros::NodeHandle nh;
  ros::Publisher points_pub, final_ground_pub, non_points_pub, floor_pub, normal_ground_pub;
  ros::Subscriber lidar_sub;

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterKeyFrames, downSizeGroundFrames, downSizeNoGroundFrames;
  std::mutex mutex_lock;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue;

  std::string cloud_topic_;

  // plane params
  double tilt_deg = 0.0;
  double sensor_height = 2.0;
  double height_clip_range = 2.5;  //[sensor_height - height_clip_range, sensor_height + height_clip_range]

  int floor_pts_thresh = 512;
  double floor_normal_thresh = 10.0;
  bool use_normal_filtering = true;
  double normal_filter_thresh = 20.0;
  bool use_outlier_removal_method = true;

  void PcCB(const sensor_msgs::PointCloud2ConstPtr &msg);

  pcl::PointCloud<PointT>::Ptr PlaneClip(const pcl::PointCloud<PointT>::Ptr &src_cloud,
                                         const Eigen::Vector4f &plane,
                                         bool negative);

  pcl::PointCloud<PointT>::Ptr NormalFiltering(const pcl::PointCloud<PointT>::Ptr &cloud);

  Eigen::Vector4f DetectPlane(const pcl::PointCloud<PointT>::Ptr &cloud, std_msgs::Header &current_header);
};

#endif //SRC_XCHU_MAPPING_SRC_POINTS_FILTER_NODE_H_
