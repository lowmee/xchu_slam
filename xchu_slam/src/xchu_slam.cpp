/**
 * @file xchu_slam.cpp
 * @author xchu
 */

#include "xchu_slam/xchu_slam.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_node");

  ros::NodeHandle nh, pnh("~");
  XCHUSLAM *slam = new XCHUSLAM(nh, pnh);

  // 这俩线程用来进行可视化和后端优化
  std::thread visual_thread(&XCHUSLAM::visualThread, slam);
  std::thread loop_thread(&XCHUSLAM::loopClosureThread, slam);

  ros::Rate rate(200);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  // 关闭终端的时候也要销毁子线程
  loop_thread.join();
  visual_thread.join();
  return 0;
}

XCHUSLAM::XCHUSLAM(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh) {
  // 各种参数初始化
  if (!init()) {
    ROS_ERROR("init failed...");
    exit(-1);
  }

  // publisher
  pub_keyposes_ = nh_.advertise<sensor_msgs::PointCloud2>("/keyposes", 1);  // 关键帧点云
  pub_globalmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/globalmap", 1);
  pub_localmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/localmap", 1);
  pub_undistorted_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/undistorted_pc", 1);
  pub_current_odom_ = nh_.advertise<nav_msgs::Odometry>("/current_odom", 1);
  pub_current_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_points", 1);

  // subscriber
  /* /vlp16_0/velodyne_points /kitti/velo/pointcloud  /lslidar_point_cloud */
  sub_pc_ =
      nh_.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 5, boost::bind(&XCHUSLAM::pcCB, this, _1));
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 5, boost::bind(&XCHUSLAM::imuCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 5, boost::bind(&XCHUSLAM::odomCB, this, _1));
}

bool XCHUSLAM::init() {
  pnh_.param<float>("scan_period", scan_period_, 0.2);
  pnh_.param<float>("keyframe_dist", keyframe_dist_, 0.3); // 用于设置关键帧之前的间距,太近就不加到位子图里面
  keyframe_dist_ *= keyframe_dist_;
  pnh_.param<float>("surround_search_radius", surround_search_radius_, 20.);
  pnh_.param<int>("surround_search_num", surround_search_num_, 50);  // localmap的帧数
  pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.5); // 用于下采样网格大小
  pnh_.param<float>("min_scan_range", min_scan_range_, 0.3);   // 太近或者太远的点云需要截掉
  min_scan_range_ *= min_scan_range_;
  pnh_.param<float>("max_scan_range", max_scan_range_, 80);
  max_scan_range_ *= max_scan_range_;
  pnh_.param<bool>("use_odom", use_odom_, false);
  pnh_.param<bool>("use_imu", use_imu_, false);
  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size, 0.1);
  pnh_.param<double>("ndt_res", ndt_res_, 2.);
  pnh_.param<int>("max_iters", max_iters_, 30);
  pnh_.param<float>("history_search_radius", history_search_radius_, 10.);  // 回环检测的候选帧搜索半径
  pnh_.param<int>("history_search_num", history_search_num_, 20);    // 选取多少历史帧
  pnh_.param<float>("history_fitness_score", history_fitness_score_, 0.3);  // 回环检测icp匹配阈值
  pnh_.param<float>("ds_history_size", ds_history_size_, 1.);
  pnh_.param<std::string>("save_dir", save_dir_, "");   // 全局地图的保存路径
  pnh_.param<bool>("loop_closure_enabled", loop_closure_enabled_, true);

  loop_closed_ = false;  // 一开始是没有闭环的

  // ndt params set
  // 注意：此处设置ndt参数之后才能赋值给registration,否则后面无法使用getFinalScore函数！
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt->setTransformationEpsilon(trans_eps_);
  ndt->setStepSize(step_size);
  ndt->setResolution(ndt_res_);
  ndt->setMaximumIterations(max_iters_);
  ndt_omp_ = ndt;

  // 初始化tf
  tf_m2o_.setIdentity();  // baselink到odom的初始变换,需要提前标定
  tf_b2l_ = Eigen::Matrix4f::Identity(); // baselink到lidar的初始变换,需要提前标定
  float roll, pitch, yaw;
  if (!nh_.getParam("tf_b2l_x", tf_b2l_(0, 3)) || !nh_.getParam("tf_b2l_y", tf_b2l_(1, 3)) ||
      !nh_.getParam("tf_b2l_z", tf_b2l_(2, 3)) || !nh_.getParam("tf_b2l_roll", roll) ||
      !nh_.getParam("tf_b2l_pitch", pitch) || !nh_.getParam("tf_b2l_yaw", yaw)) {
    ROS_ERROR("transform between /base_link to /laser not set.");
    exit(-1);
  }
  Eigen::AngleAxisf rx(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf ry(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rz(yaw, Eigen::Vector3f::UnitZ());
  tf_b2l_.block(0, 0, 3, 3) = (rz * ry * rx).matrix();

  // gtsam参数初始化
  ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam = new ISAM2(params);
  gtsam::Vector vector6(6);
  vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6; // xyz yaw是主要变化量设为一个初始数量级
  prior_noise_ = noiseModel::Diagonal::Variances(vector6);
  odom_noise_ = noiseModel::Diagonal::Variances(vector6);

  // 点云指针的初始化
  pc_source_.reset(new pcl::PointCloud<PointT>());
  pc_target_.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());  // 关键帧位置
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointXYZIRPYT>());  // 关键帧位姿
  latest_keyframe_.reset(new pcl::PointCloud<PointT>());
  near_history_keyframes_.reset(new pcl::PointCloud<PointT>()); // 回环时根据距离筛选的候选帧
  kdtree_poses_.reset(new pcl::KdTreeFLANN<PointT>());    // 在回环检测时用kdtree加速候选帧的搜索

  // 不同的下采样网格大小
  downSizeFilterSource.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  downSizeFilterHistoryKeyFrames.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  downSizeFilterGlobalMap.setLeafSize(1.0, 1.0, 1.0);

  // pose对应的T
  pre_pose_m_ = cur_pose_m_ = pre_pose_o_ = cur_pose_o_ = Eigen::Matrix4f::Identity();

  imu_ptr_front_ = odom_ptr_front_ = 0;
  imu_ptr_last_ = odom_ptr_last_ = -1;
  imu_ptr_last_iter_ = odom_ptr_last_iter_ = 0;

  imu_time_.fill(0);
  imu_roll_.fill(0);
  imu_pitch_.fill(0);
  imu_yaw_.fill(0);

  imu_acc_x_.fill(0);
  imu_acc_y_.fill(0);
  imu_acc_z_.fill(0);
  imu_velo_x_.fill(0);
  imu_velo_y_.fill(0);
  imu_velo_z_.fill(0);
  imu_shift_x_.fill(0);
  imu_shift_y_.fill(0);
  imu_shift_z_.fill(0);

  imu_angular_velo_x_.fill(0);
  imu_angular_velo_y_.fill(0);
  imu_angular_velo_z_.fill(0);
  imu_angular_rot_x_.fill(0);
  imu_angular_rot_y_.fill(0);
  imu_angular_rot_z_.fill(0);

  ROS_INFO("init.");
  return true;
}

void XCHUSLAM::visualThread() {
  // 可视化
  ros::Rate rate(0.2);
  while (ros::ok()) {
    rate.sleep();
    publishKeyposesAndFrames();
  }
  // 关闭终端保存地图
  TransformAndSaveMap();
}

void XCHUSLAM::TransformAndSaveMap() {
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
  for (int i = 0; i < cloud_keyframes_.size(); ++i) {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
    *map += *tmp;
    num_points += tmp->points.size();

    pcl::PointCloud<PointT>::Ptr tmp1(new pcl::PointCloud<PointT>());
    tmp1 = transformPointCloud(tmp1, cloud_keyposes_6d_->points[i]);
    num_points1 += tmp1->points.size();
  }

  map->width = map->points.size();
  map->height = 1;
  map->is_dense = false;

  downSizeFilterGlobalMap.setInputCloud(map);
  downSizeFilterGlobalMap.filter(*map);

  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(save_dir_ + "trajectory_" + stamp + ".pcd", *poses);
  pcl::io::savePCDFile(save_dir_ + "finalCloud_" + stamp + ".pcd", *map);

  ROS_WARN("Save map. pose size: %d, cloud size: %d, cloud no ground size: %d", poses->points.size(), num_points,
           num_points1);
}

void XCHUSLAM::adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time) {
  bool half_passed = false;
  int cloud_size = cloud->points.size();

  //  计算当前帧雷达转过的角度
  float start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
  float end_ori = -std::atan2(cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
  if (end_ori - start_ori > 3 * M_PI) {
    end_ori -= 2 * M_PI;
  } else if (end_ori - start_ori < M_PI) {
    end_ori += 2 * M_PI;
  }
  float ori_diff = end_ori - start_ori;

  Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
  Eigen::Vector3f shift_from_start;
  Eigen::Matrix3f r_s_i, r_c;
  Eigen::Vector3f adjusted_p;
  float ori_h;
  for (int i = 0; i < cloud_size; ++i) {
    PointT &p = cloud->points[i];
    ori_h = -std::atan2(p.y, p.x);
    if (!half_passed) {
      if (ori_h < start_ori - M_PI * 0.5) {
        ori_h += 2 * M_PI;
      } else if (ori_h > start_ori + M_PI * 1.5) {
        ori_h -= 2 * M_PI;
      }

      if (ori_h - start_ori > M_PI) {
        half_passed = true;
      }
    } else {
      ori_h += 2 * M_PI;
      if (ori_h < end_ori - 1.5 * M_PI) {
        ori_h += 2 * M_PI;
      } else if (ori_h > end_ori + 0.5 * M_PI) {
        ori_h -= 2 * M_PI;
      }
    }

    float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

    if (imu_ptr_last_ > 0) {
      imu_ptr_front_ = imu_ptr_last_iter_;
      while (imu_ptr_front_ != imu_ptr_last_) {
        if (scan_time + rel_time < imu_time_[imu_ptr_front_]) {
          break;
        }
        imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
      }
      if (std::abs(scan_time + rel_time - imu_time_[imu_ptr_front_]) > scan_period_) {
        ROS_WARN_COND(i < 10, "unsync imu and pc msg");
        continue;
      }

      if (scan_time + rel_time > imu_time_[imu_ptr_front_]) {
        rpy_cur(0) = imu_roll_[imu_ptr_front_];
        rpy_cur(1) = imu_pitch_[imu_ptr_front_];
        rpy_cur(2) = imu_yaw_[imu_ptr_front_];
        shift_cur(0) = imu_shift_x_[imu_ptr_front_];
        shift_cur(1) = imu_shift_y_[imu_ptr_front_];
        shift_cur(2) = imu_shift_z_[imu_ptr_front_];
        velo_cur(0) = imu_velo_x_[imu_ptr_front_];
        velo_cur(1) = imu_velo_y_[imu_ptr_front_];
        velo_cur(2) = imu_velo_z_[imu_ptr_front_];
      } else {
        int imu_ptr_back = (imu_ptr_front_ - 1 + imu_queue_len_) % imu_queue_len_;
        float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) /
            (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
        float ratio_back = 1. - ratio_front;
        rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] * ratio_back;
        rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] * ratio_back;
        rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
        shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] * ratio_back;
        shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] * ratio_back;
        shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] * ratio_back;
        velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] * ratio_back;
        velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] * ratio_back;
        velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] * ratio_back;
      }

      r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

      if (i == 0) {
        rpy_start = rpy_cur;
        shift_start = shift_cur;
        velo_start = velo_cur;
        r_s_i = r_c.inverse();
      } else {
        shift_from_start = shift_cur - shift_start - velo_start * rel_time;
        adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
        p.x = adjusted_p.x();
        p.y = adjusted_p.y();
        p.z = adjusted_p.z();
      }
    }
    imu_ptr_last_iter_ = imu_ptr_front_;
  }

  // 发布去完畸变的点云帧
  if (pub_undistorted_pc_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp.fromSec(scan_time);
    msg.header.frame_id = "/laser";
    pub_undistorted_pc_.publish(msg);
  }
}

void XCHUSLAM::extractSurroundKeyframes() {

  // cloud_keyframes_用来存储关键帧, localmap是选取较近的关键帧来进行不断更新的
  if (cloud_keyframes_.empty()) {
    return;
  }

  bool target_updated = false;
  if (recent_keyframes_.size() < surround_search_num_) {
    // localmap的帧数较少,加入新的关键这帧
    recent_keyframes_.clear();
    for (int i = cloud_keyposes_3d_->points.size() - 1; i >= 0; --i) {
      int this_key_id = int(cloud_keyposes_3d_->points[i].intensity);
      pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
      tf_cloud = transformPointCloud(cloud_keyframes_[this_key_id], cloud_keyposes_6d_->points[this_key_id]);
      recent_keyframes_.push_back(tf_cloud);
      // 这里可以缩减一下时间
      if (recent_keyframes_.size() >= surround_search_num_) {
        break;
      }
    }
    target_updated = true;
  } else {
    // localmap的帧数够的话,去掉旧帧, 加入新帧
    static int latest_frame_id = cloud_keyframes_.size() - 1;
    if (latest_frame_id != cloud_keyframes_.size() - 1) {
      latest_frame_id = cloud_keyframes_.size() - 1;
      recent_keyframes_.pop_back();
      pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
      tf_cloud = transformPointCloud(cloud_keyframes_[latest_frame_id],
                                     cloud_keyposes_6d_->points[latest_frame_id]);
      recent_keyframes_.push_front(tf_cloud);
      target_updated = true;
    }
  }

  // 将localmap作为匹配的target
  if (target_updated) {
    pc_target_->clear();
    for (auto keyframe : recent_keyframes_) {
      *pc_target_ += *keyframe;
    }
    ndt_omp_->setInputTarget(pc_target_);
    //ROS_INFO("update localmap...");
  }
}

bool XCHUSLAM::saveKeyframesAndFactor() {
  // 此处的当前位姿(cur_pose_ndt_)为ndt匹配后的final_transformation
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y,
                               cur_pose_ndt_.pose.pose.orientation.z, cur_pose_ndt_.pose.pose.orientation.w)).getRPY(
      roll, pitch, yaw);

  if (cloud_keyposes_3d_->points.empty()) {
    // gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    // 第一帧进来添加先验因子并初始化因子图
    // 一元因子, 系统先验
    gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(roll, pitch, yaw),
                                                Point3(cur_pose_ndt_.pose.pose.position.x,
                                                       cur_pose_ndt_.pose.pose.position.y,
                                                       cur_pose_ndt_.pose.pose.position.z)), prior_noise_));

    // 给变量0赋值(位姿)
    initial_estimate_.insert(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w,
                                                       cur_pose_ndt_.pose.pose.orientation.x,
                                                       cur_pose_ndt_.pose.pose.orientation.y,
                                                       cur_pose_ndt_.pose.pose.orientation.z),
                                      Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y,
                                             cur_pose_ndt_.pose.pose.position.z)));
    pre_keypose_ = cur_pose_ndt_;
  } else {
    const auto &pre_pose = cloud_keyposes_6d_->points[cloud_keyposes_3d_->points.size() - 1];
    if (std::pow(cur_pose_ndt_.pose.pose.position.x - pre_pose.x, 2) +
        std::pow(cur_pose_ndt_.pose.pose.position.y - pre_pose.y, 2) +
        std::pow(cur_pose_ndt_.pose.pose.position.z - pre_pose.z, 2) <
        keyframe_dist_) {
      // 关键帧之间驱离太近则不加入因子
      return false;
    }

    // pitch视作不参与优化
    // gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(pre_keypose_.pose.pose.orientation.w, pre_keypose_.pose.pose.orientation.x, pre_keypose_.pose.pose.orientation.y, pre_keypose_.pose.pose.orientation.z), Point3(pre_keypose_.pose.pose.position.x, pre_keypose_.pose.pose.position.y, pre_keypose_.pose.pose.position.z));
    gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(pre_pose.roll, pre_pose.pitch, pre_pose.yaw),
                                   Point3(pre_pose.x, pre_pose.y, pre_pose.z));
    // gtsam::Pose3 pose_to = Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z));
    gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw),
                                 Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y,
                                        cur_pose_ndt_.pose.pose.position.z * 0));
    // 二元因子, 添加回环之间的关系
    gtSAMgraph_.add(BetweenFactor<Pose3>(cloud_keyposes_3d_->points.size() - 1, cloud_keyposes_3d_->points.size(),
                                         pose_from.between(pose_to), odom_noise_));
    // 给当前变量赋值
    initial_estimate_.insert(cloud_keyposes_3d_->points.size(), Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw),
                                                                      Point3(cur_pose_ndt_.pose.pose.position.x,
                                                                             cur_pose_ndt_.pose.pose.position.y,
                                                                             cur_pose_ndt_.pose.pose.position.z *
                                                                                 0)));
  }

  // 更新isam
  isam->update(gtSAMgraph_, initial_estimate_);
  isam->update();

  // 清除graph和initial value, 否则会重复更新
  gtSAMgraph_.resize(0);
  initial_estimate_.clear();

  PointT this_pose_3d;
  PointXYZIRPYT this_pose_6d;

  // 提取优化结果, 更新关键帧及其位姿
  Pose3 latest_estimate;
  isam_current_estimate_ = isam->calculateEstimate();
  latest_estimate = isam_current_estimate_.at<Pose3>(isam_current_estimate_.size() - 1);// 最新的一个节点

  this_pose_6d.x = this_pose_3d.x = latest_estimate.translation().x();
  this_pose_6d.y = this_pose_3d.y = latest_estimate.translation().y();
  this_pose_6d.z = this_pose_3d.z = latest_estimate.translation().z();
  // 注意其序号赋值给intensity字段
  this_pose_6d.intensity = this_pose_3d.intensity = cloud_keyposes_3d_->points.size();
  this_pose_6d.roll = latest_estimate.rotation().roll();
  this_pose_6d.pitch = latest_estimate.rotation().pitch();
  this_pose_6d.yaw = latest_estimate.rotation().yaw();
  this_pose_6d.time = cur_pose_ndt_.header.stamp.toSec();
  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyposes_6d_->points.push_back(this_pose_6d);

  if (cloud_keyposes_3d_->points.size() > 1) {
    pre_keypose_.pose.pose.position.x = this_pose_3d.x;
    pre_keypose_.pose.pose.position.y = this_pose_3d.y;
    pre_keypose_.pose.pose.position.z = this_pose_3d.z;
    pre_keypose_.pose.pose.orientation.w = latest_estimate.rotation().toQuaternion().w();
    pre_keypose_.pose.pose.orientation.x = latest_estimate.rotation().toQuaternion().x();
    pre_keypose_.pose.pose.orientation.y = latest_estimate.rotation().toQuaternion().y();
    pre_keypose_.pose.pose.orientation.z = latest_estimate.rotation().toQuaternion().z();
    pre_keypose_.header.stamp = cur_pose_ndt_.header.stamp;
  }

  cur_pose_m_.block<3, 3>(0, 0) = Eigen::Quaternionf(pre_keypose_.pose.pose.orientation.w,
                                                     pre_keypose_.pose.pose.orientation.x,
                                                     pre_keypose_.pose.pose.orientation.y,
                                                     pre_keypose_.pose.pose.orientation.z).toRotationMatrix();
  cur_pose_m_(0, 3) = pre_keypose_.pose.pose.position.x;
  cur_pose_m_(1, 3) = pre_keypose_.pose.pose.position.y;
  cur_pose_m_(2, 3) = pre_keypose_.pose.pose.position.z;

  // 添加关键帧
  pcl::PointCloud<PointT>::Ptr cur_keyframe(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*pc_source_, *cur_keyframe);
  for (auto &p : cur_keyframe->points) {
    p.intensity = this_pose_3d.intensity;
  }
  cloud_keyframes_.push_back(cur_keyframe);

  ROS_INFO("saveKeyframesAndFactor: %d points", cur_keyframe->points.size());

  return true;
}

void XCHUSLAM::publishKeyposesAndFrames() {
  // 发布优化后的关键帧位置点
  if (pub_keyposes_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_keyposes_.publish(msg);
  }

  // 发布全局地图,这里为了不卡,对地图进行下采样
  if (pub_globalmap_.getNumSubscribers() > 0) {
    pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
    // 把栈里面所有的关键帧拿出来拼成地图
    for (int i = 0; i < cloud_keyframes_.size(); ++i) {
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
      tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
      *map += *tmp;
    }

    // downsample visualized points
    downSizeFilterGlobalMap.setInputCloud(map);
    downSizeFilterGlobalMap.filter(*map);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_globalmap_.publish(msg);
  }
}

void XCHUSLAM::odomCB(const nav_msgs::OdometryConstPtr &msg) {
  // 里程计数据的队列长度也是100
  odom_ptr_last_ = (odom_ptr_last_ + 1) % imu_queue_len_;
  if ((odom_ptr_last_ + 1) % imu_queue_len_ == odom_ptr_front_) {
    odom_ptr_front_ = (odom_ptr_front_ + 1) % imu_queue_len_;
  }
  // 将数据装到队列里面
  odom_queue_[odom_ptr_last_] = *msg;
}

void XCHUSLAM::pcCB(const sensor_msgs::PointCloud2ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  auto start = std::chrono::system_clock::now();

  //  提取附近的点云帧
  extractSurroundKeyframes();

  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*msg, *tmp_cloud);

//  std::vector<int> indices;   //remove NAN
//  pcl::removeNaNFromPointCloud(tmp_cloud, tmp_cloud, indices);

  // 点云转换到/base_link坐标系中处理
  pc_source_->clear();
  pcl::transformPointCloud(*tmp_cloud, *pc_source_, tf_b2l_);

  // 去运动畸变
  if (use_imu_) {
    adjustDistortion(pc_source_, msg->header.stamp.toSec());
  }

  tmp_cloud->clear();
  downSizeFilterSource.setInputCloud(pc_source_);
  downSizeFilterSource.filter(*tmp_cloud);
  pc_source_->clear();

  float r;
  for (const auto &p : tmp_cloud->points) {
    r = p.x * p.x + p.y * p.y;
    if (r > min_scan_range_ && r < max_scan_range_) {
      pc_source_->points.push_back(p);
    }
  }

  // 第一帧点云存为target, 初始化起始位置
  if (cloud_keyframes_.empty()) {
    ROS_INFO("first laser frame.");
    *pc_target_ += *pc_source_;
    ndt_omp_->setInputTarget(pc_target_);

    // 使用odom做初值
    if (use_odom_ && odom_ptr_last_ != -1) {
      int odom_ptr = odom_ptr_front_;
      while (odom_ptr != odom_ptr_last_) {
        // 在odom队列中寻找时间距离最近的帧
        if (odom_queue_[odom_ptr].header.stamp > msg->header.stamp) {
          break;
        }
        odom_ptr = (odom_ptr + 1) % imu_queue_len_;
      }
      // 更新pre_pose_o_
      pre_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.x,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.y,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
      pre_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
      pre_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
      pre_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;

      // 更新cur_pos
      cur_pose_o_ = pre_pose_o_;
      odom_ptr_front_ = odom_ptr; // 更新指针
    }
  }

  // 初始位姿估计
  if (use_odom_ && odom_ptr_last_ != -1) {
    pre_pose_o_ = cur_pose_o_;
    // 寻找最近帧
    int odom_ptr = odom_ptr_front_;
    while (odom_ptr != odom_ptr_last_) {
      if (odom_queue_[odom_ptr].header.stamp > msg->header.stamp) {
        break;
      }
      odom_ptr = (odom_ptr + 1) % imu_queue_len_;
    }
    // 直接赋值给cur_pose_o_
    cur_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.x,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.y,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
    cur_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
    cur_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
    cur_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;
    odom_ptr_front_ = odom_ptr; // 更新指针

    Eigen::Quaternionf tmp_q(pre_pose_m_.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
    pre_pose_m_.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Matrix4f r_m2o = Eigen::Matrix4f::Identity();
    r_m2o.block<3, 3>(0, 0) = Eigen::Quaternionf(tf_m2o_.getRotation().w(), tf_m2o_.getRotation().x(),
                                                 tf_m2o_.getRotation().y(),
                                                 tf_m2o_.getRotation().z()).toRotationMatrix();
    cur_pose_m_ = pre_pose_m_ * pre_pose_o_.inverse() * cur_pose_o_; // 预测当前位姿

    tmp_q = Eigen::Quaternionf(cur_pose_m_.block<3, 3>(0, 0));
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
  } else {
    // 纯雷达模型
    predict_pose_ = pre_pose_ndt_;
    cur_pose_m_ = pre_pose_m_;
  }

  // ndt匹配
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud_(new pcl::PointCloud<pcl::PointXYZI>);
  ndt_omp_->setInputSource(pc_source_);
  ndt_omp_->align(*aligned_cloud_, cur_pose_m_);
  fitness_score_ = ndt_omp_->getFitnessScore();
  final_transformation_ = ndt_omp_->getFinalTransformation();
  has_converged_ = ndt_omp_->hasConverged();
  final_iters_ = ndt_omp_->getFinalNumIteration();

  // 匹配的结果是一个相对位姿
  Eigen::Quaternionf tmp_q(final_transformation_.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  cur_pose_ndt_.pose.pose.position.x = final_transformation_(0, 3);
  cur_pose_ndt_.pose.pose.position.y = final_transformation_(1, 3);
  cur_pose_ndt_.pose.pose.position.z = final_transformation_(2, 3);
  cur_pose_ndt_.pose.pose.orientation.w = tmp_q.w();
  cur_pose_ndt_.pose.pose.orientation.x = tmp_q.x();
  cur_pose_ndt_.pose.pose.orientation.y = tmp_q.y();
  cur_pose_ndt_.pose.pose.orientation.z = tmp_q.z();
  cur_pose_ndt_.header.stamp = msg->header.stamp;
  cur_pose_m_ = final_transformation_;

  // publish odom
  if (pub_current_odom_.getNumSubscribers() > 0) {
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = final_transformation_(0, 3);
    odom.pose.pose.position.y = final_transformation_(1, 3);
    odom.pose.pose.position.z = final_transformation_(2, 3);

    odom.pose.pose.orientation.x = tmp_q.x();
    odom.pose.pose.orientation.y = tmp_q.y();
    odom.pose.pose.orientation.z = tmp_q.z();
    odom.pose.pose.orientation.w = tmp_q.w();

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    pub_current_odom_.publish(odom);
  }

  // 这样写tf的话, 在rviz中可视化的时候,界面是不会动的
  tf::Transform tf_m2b;
  tf_m2b.setOrigin(
      tf::Vector3(final_transformation_(0, 3), final_transformation_(1, 3), final_transformation_(2, 3)));
  tf_m2b.setRotation(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w()));

  if (use_odom_ && odom_ptr_last_ != -1) {
    // 如果使用odom的话需要考虑odom到base_link的变换
    tf_o2b_.setOrigin(tf::Vector3(odom_queue_[odom_ptr_front_].pose.pose.position.x,
                                  odom_queue_[odom_ptr_front_].pose.pose.position.y,
                                  odom_queue_[odom_ptr_front_].pose.pose.position.z));
    tf_o2b_.setRotation(tf::Quaternion(odom_queue_[odom_ptr_front_].pose.pose.orientation.x,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.y,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.z,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.w));
    tf_m2o_ = tf_m2b * tf_o2b_.inverse();
    // tf::StampedTransform tmp;
    // tf_listener_.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.05));
    // tf_listener_.lookupTransform("/odom", "/base_link", ros::Time(0), tmp);
    // tf_m2o_ = tf_m2b * tmp.inverse();
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2o_, msg->header.stamp, "map", "/odom"));
  } else {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2b, msg->header.stamp, "map", "/base_link"));
  }

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Number of scan points: " << pc_source_->size() << " points." << std::endl;
  std::cout << "global map: " << pc_target_->points.size() << " points." << std::endl;
  std::cout << "Fitness score: " << fitness_score_ << std::endl;
  std::cout << "Number of iteration: " << final_iters_ << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << final_transformation_ << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(final_transformation_(0, 3) - pre_pose_m_(0, 3), 2) +
      std::pow(final_transformation_(1, 3) - pre_pose_m_(1, 3), 2) +
      std::pow(final_transformation_(2, 3) - pre_pose_m_(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  // 保存关键帧和因子
  if (saveKeyframesAndFactor()) {
    pcl::PointCloud<PointT>::Ptr pc_m(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*pc_source_, *pc_m, cur_pose_m_);

    // 实时的点云也发布
    downSizeFilterSource.setInputCloud(pc_m);
    downSizeFilterSource.filter(*pc_m);
    if (pub_current_points_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2::Ptr pointcloud_current_ptr(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*pc_m, *pointcloud_current_ptr);
      pointcloud_current_ptr->header.frame_id = "map";
      pub_current_points_.publish(*pointcloud_current_ptr);
    }
    pc_m->clear();

    // localmap
    pcl::PointCloud<PointT>::Ptr local_cloud(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*pc_target_, *local_cloud);
    downSizeFilterSource.setInputCloud(local_cloud);
    downSizeFilterSource.filter(*local_cloud);
    if (pub_localmap_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2::Ptr localmap_msg_ptr(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*local_cloud, *localmap_msg_ptr);
      localmap_msg_ptr->header.frame_id = "map";
      localmap_msg_ptr->header.stamp = ros::Time::now();
      pub_localmap_.publish(*localmap_msg_ptr);
    }
    local_cloud->clear();
  } else {
    // std::cout << "too close" << std::endl;
  }

  pre_pose_m_ = cur_pose_m_;
  pre_pose_ndt_ = cur_pose_ndt_;

  // 匹配完了去更新当前pose
  correctPoses();
}

void XCHUSLAM::imuCB(const sensor_msgs::ImuConstPtr &msg) {

  //  取姿态，是ENU数据吗？
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = msg->linear_acceleration.x + 9.81 * sin(pitch);
  float acc_y = msg->linear_acceleration.y - 9.81 * cos(pitch) * sin(roll);
  float acc_z = msg->linear_acceleration.z - 9.81 * cos(pitch) * cos(roll);

  // imu队列里面装100条数据
  imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_queue_len_;

  if ((imu_ptr_last_ + 1) % imu_queue_len_ == imu_ptr_front_) {
    imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
  }

  // 维护几个队列的imu数据
  imu_time_[imu_ptr_last_] = msg->header.stamp.toSec();
  imu_roll_[imu_ptr_last_] = roll;
  imu_pitch_[imu_ptr_last_] = pitch;
  imu_yaw_[imu_ptr_last_] = yaw;
  imu_acc_x_[imu_ptr_last_] = acc_x;
  imu_acc_y_[imu_ptr_last_] = acc_y;
  imu_acc_z_[imu_ptr_last_] = acc_z;
  imu_angular_velo_x_[imu_ptr_last_] = msg->angular_velocity.x;
  imu_angular_velo_y_[imu_ptr_last_] = msg->angular_velocity.y;
  imu_angular_velo_z_[imu_ptr_last_] = msg->angular_velocity.z;

  // 转换到 imu 的全局坐标系中
  Eigen::Matrix3f rot = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y,
                                           msg->orientation.z).toRotationMatrix();
  Eigen::Vector3f acc = rot * Eigen::Vector3f(acc_x, acc_y, acc_z);
  // TODO: lego_loam 里没有对角速度转换，是否需要尚且存疑
  // Eigen::Vector3f angular_velo = rot * Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3f angular_velo(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  //  这里是记录imu在这个队列数据时间内的偏移？
  int imu_ptr_back = (imu_ptr_last_ - 1 + imu_queue_len_) % imu_queue_len_;
  double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
  if (time_diff < 1.) {
    imu_shift_x_[imu_ptr_last_] = imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff +
        acc(0) * time_diff * time_diff * 0.5;
    imu_shift_y_[imu_ptr_last_] = imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff +
        acc(1) * time_diff * time_diff * 0.5;
    imu_shift_z_[imu_ptr_last_] = imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff +
        acc(2) * time_diff * time_diff * 0.5;

    imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
    imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
    imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

    imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) * time_diff;
    imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) * time_diff;
    imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) * time_diff;
  }
}

void XCHUSLAM::loopClosureThread() {
  // 不停地进行回环检测
  ros::Duration duration(1);
  while (ros::ok()) {
    performLoopClosure();
    duration.sleep();
  }
}

/**
 * @brief 回环检测及位姿图更新
 * ICP 匹配添加回环约束
 */
void XCHUSLAM::performLoopClosure() {
  if (cloud_keyposes_3d_->points.empty()) {
    return;
  }

  if (!detectLoopClosure()) {
    return;
  } else {
    ROS_WARN("detected loop closure");
  }

  // 检测到回环后，用icp去匹配
  auto start = std::chrono::system_clock::now();

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(latest_keyframe_);
  icp.setInputTarget(near_history_keyframes_);
  pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
  Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
  initial_guess.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].roll, Eigen::Vector3f::UnitX()))
      .toRotationMatrix();
  initial_guess(0, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].x;
  initial_guess(1, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].y;
  initial_guess(2, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].z;
  icp.align(*unused_result);

  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();
  Eigen::Matrix4f correction_frame = icp.getFinalTransformation();
  Eigen::Quaternionf tmp_q(correction_frame.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  if (has_converged == false || fitness_score > history_fitness_score_) {
    ROS_WARN("loop cannot closed");
    return;
  } else {
    ROS_WARN("loop closed");
  }

  Eigen::Matrix4f t_wrong = initial_guess;
  Eigen::Matrix4f t_correct = correction_frame * t_wrong;
  // Eigen::Matrix4f t_correct = correction_frame;
  Eigen::Quaternionf r_correct(t_correct.block<3, 3>(0, 0));
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(r_correct.w(), r_correct.x(), r_correct.y(), r_correct.z()),
                                 Point3(t_correct(0, 3), t_correct(1, 3), t_correct(2, 3)));
  gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(cloud_keyposes_6d_->points[closest_history_frame_id_].roll,
                                            cloud_keyposes_6d_->points[closest_history_frame_id_].pitch,
                                            cloud_keyposes_6d_->points[closest_history_frame_id_].yaw),
                               Point3(cloud_keyposes_6d_->points[closest_history_frame_id_].x,
                                      cloud_keyposes_6d_->points[closest_history_frame_id_].y,
                                      cloud_keyposes_6d_->points[closest_history_frame_id_].z));
  float noise_score = fitness_score;
  gtsam::Vector vector6(6);
  vector6 << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;
  constraint_noise_ = noiseModel::Diagonal::Variances(vector6);

  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph_.add(
      BetweenFactor<Pose3>(latest_history_frame_id_, closest_history_frame_id_, pose_from.between(pose_to),
                           constraint_noise_));
  isam->update(gtSAMgraph_);
  isam->update();
  gtSAMgraph_.resize(0);

  double shift = std::sqrt(std::pow(correction_frame(0, 3) - initial_guess(0, 3), 2) +
      std::pow(correction_frame(1, 3) - initial_guess(1, 3), 2) +
      std::pow(correction_frame(2, 3) - initial_guess(2, 3), 2));
  std::cout << "----------------------------loop clouser-------------------------------------" << std::endl;
  std::cout << "Number of source points: " << latest_keyframe_->size() << " points." << std::endl;
  std::cout << "target: " << near_history_keyframes_->points.size() << " points." << std::endl;
  std::cout << "ICP has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_correct << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "----------------------------loop clouser-------------------------------------" << std::endl;

  loop_closed_ = true;
}

/**
 * @brief 基于里程计的回环检测，直接提取当前位姿附近(radiusSearch)的 keyframe 作为 icp 的 target
 * 
 */
bool XCHUSLAM::detectLoopClosure() {
  latest_keyframe_->clear();
  near_history_keyframes_->clear();

  std::lock_guard<std::mutex> lock(mtx_);

  PointT cur_pose;
  cur_pose.x = cur_pose_m_(0, 3);
  cur_pose.y = cur_pose_m_(1, 3);
  cur_pose.z = cur_pose_m_(2, 3);
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  kdtree_poses_->radiusSearch(cur_pose, history_search_radius_, search_idx_, search_dist_);

  latest_history_frame_id_ = cloud_keyframes_.size() - 1;
  closest_history_frame_id_ = -1;
  for (int i = 0; i < search_idx_.size(); ++i) {
    if (cur_pose_ndt_.header.stamp.toSec() - cloud_keyposes_6d_->points[search_idx_[i]].time > 30.) {
      closest_history_frame_id_ = search_idx_[i];
      break;
    }
  }
  // 时间太短不做回环
  if (closest_history_frame_id_ == -1) {
    return false;
  }

  pcl::copyPointCloud(*transformPointCloud(cloud_keyframes_[latest_history_frame_id_],
                                           cloud_keyposes_6d_->points[latest_history_frame_id_]), *latest_keyframe_);

  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i) {
    j = closest_history_frame_id_ + i;
    if (j < 0 || j >= latest_history_frame_id_) {
      continue;
    }
    *tmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
  }

  downSizeFilterHistoryKeyFrames.setInputCloud(tmp_cloud);
  downSizeFilterHistoryKeyFrames.filter(*near_history_keyframes_);

  return true;
}

/**
 * @brief 检测出回环后，更新位姿及 target map
 * 
 */
void XCHUSLAM::correctPoses() {
  if (loop_closed_) {
    recent_keyframes_.clear();
    ROS_WARN("correctPoses");
    int num_poses = isam_current_estimate_.size();
    for (int i = 0; i < num_poses; ++i) {
      // if (std::abs(cloud_keyposes_3d_->points[i].z - isam_current_estimate_.at<Pose3>(i).translation().z()) > 1)
      // {
      //   ROS_WARN("aaaa");
      // }
      // else
      // {
      //   ROS_WARN("bbbb");
      // }
      cloud_keyposes_6d_->points[i].x = cloud_keyposes_3d_->points[i].x = isam_current_estimate_.at<Pose3>(
          i).translation().x();
      cloud_keyposes_6d_->points[i].y = cloud_keyposes_3d_->points[i].y = isam_current_estimate_.at<Pose3>(
          i).translation().y();
      cloud_keyposes_6d_->points[i].z = cloud_keyposes_3d_->points[i].z = isam_current_estimate_.at<Pose3>(
          i).translation().z();
      cloud_keyposes_6d_->points[i].roll = isam_current_estimate_.at<Pose3>(i).rotation().roll();
      cloud_keyposes_6d_->points[i].pitch = isam_current_estimate_.at<Pose3>(i).rotation().pitch();
      cloud_keyposes_6d_->points[i].yaw = isam_current_estimate_.at<Pose3>(i).rotation().yaw();
    }

    loop_closed_ = false;
  }
}

