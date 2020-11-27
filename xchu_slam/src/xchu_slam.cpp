/**
 * @file xchu_slam.cpp
 * @author xchu
 * @date 2020-10-20
 */

#include "xchu_slam/xchu_slam.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_node");

  ros::NodeHandle nh, pnh("~");
  XCHUSlam *slam = new XCHUSlam(nh, pnh);

  std::thread visual_thread(&XCHUSlam::visualThread, slam);
  std::thread loop_thread(&XCHUSlam::loopClosureThread, slam);

  ros::Rate rate(200);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  // 关闭终端时保存地图
  slam->transformAndSaveFinalMap();

  // 阻塞进程，回收资源
  loop_thread.join();
  visual_thread.join();

  return 0;
}

XCHUSlam::XCHUSlam(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh) {
  // 初始化gtsam参数
  if (!init()) {
    exit(-1);
  }

  pub_keyposes_ = nh_.advertise<sensor_msgs::PointCloud2>("/keyposes", 1);  // 关键帧点云
  pub_globalmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/globalmap", 1);  // 提取到的附近点云
  pub_predict_odom_ = nh_.advertise<nav_msgs::Odometry>("/predict_odom", 1); //  初始位姿态估计
  pub_final_odom_ = nh_.advertise<nav_msgs::Odometry>("/final_odom", 1); //  优化之后的pose
  pub_localmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/localmap", 1);
  pub_current_frames_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_frame", 1);
  pub_undistorted_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/undistorted_pc", 1);

  sub_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>(_lidar_topic.c_str(), 100, boost::bind(&XCHUSlam::pcCB, this, _1));
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(_imu_topic.c_str(), 5000, boost::bind(&XCHUSlam::imuCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(_odom_topic.c_str(), 5000, boost::bind(&XCHUSlam::odomCB, this, _1));
  sub_gps_ = nh_.subscribe<nav_msgs::Odometry>("/gps_odometry", 200, boost::bind(&XCHUSlam::gpsCB, this, _1));
}

bool XCHUSlam::init() {
  pnh_.param<float>("scan_period", scan_period_, 0.1);
  pnh_.param<float>("keyframe_dist", keyframe_dist_, 0.5);
  // pnh_.param<float>("surround_search_radius", surround_search_radius_, 20);
  pnh_.param<int>("surround_search_num", surround_search_num_, 15);
  pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.5);
  pnh_.param<float>("min_scan_range", min_scan_range_, 2);
  min_scan_range_ *= min_scan_range_;
  pnh_.param<float>("max_scan_range", max_scan_range_, 80);
  pnh_.param<double>("max_localmap_size", max_localmap_size, 30);
  pnh_.param<double>("min_add_scan_shift", min_add_scan_shift, 0.5);
  max_scan_range_ *= max_scan_range_;

  pnh_.param<bool>("use_odom", use_odom_, true);
  pnh_.param<bool>("use_imu", use_imu_, true);
  pnh_.param<bool>("use_gps", use_gps_, true);

  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size, 0.1);
  pnh_.param<double>("ndt_res", ndt_res_, 2.0);
  pnh_.param<int>("max_iters", max_iters_, 40);

  pnh_.param<float>("history_search_radius", history_search_radius_, 20);
  pnh_.param<int>("history_search_num", history_search_num_, 15);
  pnh_.param<float>("history_fitness_score", history_fitness_score_, 0.5);
  pnh_.param<float>("ds_history_size", ds_history_size_, 1);

  pnh_.param<std::string>("save_dir", save_dir_, "");

  pnh_.param<std::string>("imu_topic", _imu_topic, "/imu_raw");
  pnh_.param<std::string>("odom_topic", _odom_topic, "/odom_raw");
  pnh_.param<std::string>("lidar_topic", _lidar_topic, "/velodyne_points");
  pnh_.param<std::string>("gps_topic", _gps_topic, "/velodyne_points");

  std::cout << "imu topic: " << _imu_topic << std::endl;
  std::cout << "odom topic: " << _odom_topic << std::endl;
  std::cout << "lidar topic: " << _lidar_topic << std::endl;
  std::cout << "gps topic: " << _gps_topic << std::endl;
  std::cout << "sav dir: " << save_dir_ << std::endl;
  std::cout << "use imu: " << use_imu_ << std::endl;
  std::cout << "use odom: " << use_odom_ << std::endl;
  std::cout << "use gps: " << use_gps_ << std::endl;
  std::cout << std::endl;

  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  //    cpu_ndt_.setTransformationEpsilon(trans_eps_);
  //    cpu_ndt_.setResolution(ndt_res_);
  //    cpu_ndt_.setStepSize(step_size);
  //    cpu_ndt_.setMaximumIterations(max_iters_);
  ndt->setTransformationEpsilon(trans_eps_);
  ndt->setResolution(ndt_res_);
  ndt->setStepSize(step_size);
  //  ndt->setMaximumIterations(max_iters_);
  ndt_omp_ = ndt;

  // 初始化tf
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

  // 不同的下采样网格大小
  downSizeFilterGlobalMapKeyFrames.setLeafSize(1.0, 1.0, 1.0); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterGlobalMap.setLeafSize(1.0, 1.0, 1.0); // 保存全局地图时下采样size，可以设置小一些方便看清楚点云细节
  downSizeFilterLocalmap.setLeafSize(0.5, 0.5, 0.5);// 发布localmap的下采样size
  downSizeFilterSource.setLeafSize(0.5, 0.5, 0.5);  // 实时点云帧滤波的size
  downSizeFilterHistoryKeyframes.setLeafSize(0.5, 0.5, 0.5); // 回环检测时回环历史帧localmap下采样大小

  // gtsam参数初始化
  ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam = new ISAM2(params);
  gtsam::Vector vector6(6);
  vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  prior_noise_ = noiseModel::Diagonal::Variances(vector6);
  odom_noise_ = noiseModel::Diagonal::Variances(vector6);

  tf_m2o_.setIdentity();

  // 地面去除的设定
  ground_filter.setIfClipHeight(false);
  ground_filter.setMinDistance(1.0);

  // 初始化点云相关变量
  pc_source_.reset(new pcl::PointCloud<PointT>());
  pc_target_.reset(new pcl::PointCloud<PointT>());

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointXYZIRPYT>());

  latest_keyframe_.reset(new pcl::PointCloud<PointT>());
  near_history_keyframes_.reset(new pcl::PointCloud<PointT>());

  kdtree_poses_.reset(new pcl::KdTreeFLANN<PointT>());

  useImuHeadingInitialization = false;
  useGpsElevation = false;
  gpsCovThreshold = 2.0;
  poseCovThreshold = 10.0;

  // imu指针
  imu_ptr_front_ = 0;
  imu_ptr_last_ = -1;
  imu_ptr_last_iter_ = 0;

  // imu队列
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

  ROS_INFO("Init params.");
  return true;
}

void XCHUSlam::odomCB(const nav_msgs::OdometryConstPtr &msg) {
  odom = *msg;
  odomCalc(msg->header.stamp);
}

void XCHUSlam::pcCB(const sensor_msgs::PointCloud2ConstPtr &msg) {

  // 等待GNSS初始化全局位姿
  if (!system_initialized_) {
    ROS_WARN("Waiting for system initialized..");
    return;
  }

  pcl::PointCloud<PointT> tmp;
  pcl::fromROSMsg(*msg, tmp);
  if (tmp.empty()) {
    ROS_ERROR("Waiting for point cloud...");
    return;
  }
  std::lock_guard<std::mutex> lock(mtx_);

  // 点云简单过滤
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(tmp, tmp, indices);
  pcl::PointCloud<PointT> filter_cloud;
  for (const auto &p : tmp.points) {
    float r = p.x * p.x + p.y * p.y;
    if (r > min_scan_range_ && r < max_scan_range_) {
      filter_cloud.points.push_back(p);
    }
  }

  // 提取附近的点云帧，用一个栈来实时维护localamp
  //  extractLocalmap();

  // 点云匹配
  processCloud(filter_cloud, msg->header.stamp);

  // 选取关键帧, 更新位姿图
  saveKeyframesAndFactor();

  updateLocalmap();

  // 检测到回环, 修正pose
  correctPoses();
}

void XCHUSlam::pcCB2(const sensor_msgs::PointCloud2ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  mutex_lock.lock();
  cloudBuf.push(*msg);
  mutex_lock.unlock();
}

void XCHUSlam::imuCB(const sensor_msgs::ImuConstPtr &msg) {
//  std::lock_guard<std::mutex> lock(mtx_);
  if (imu_upside_down_)  // _imu_upside_down指示是否进行imu的正负变换
    imuUpSideDown(const_cast<sensor_msgs::Imu &>(*msg));

  //  imu_data.push_back(msg);
  //  mutex_lock.lock();
  //imuBuf.push(*input);
  //  mutex_lock.unlock();

  const ros::Time current_time = msg->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  // 解析imu消息,获得rpy
  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(msg->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = warpToPmPi(imu_roll);  // 调整,防止超过PI(180°)  --保持在±180°内
  imu_pitch = warpToPmPi(imu_pitch);
  imu_yaw = warpToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  imu.header = msg->header;
  imu.linear_acceleration.x = msg->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0) {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  } else {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imuCalc(msg->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;


  //  取姿态
//  double roll, pitch, yaw;
//  tf::Quaternion orientation;
//  tf::quaternionMsgToTF(msg->orientation, orientation);
//  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  // 加速度
  float acc_x = msg->linear_acceleration.x + 9.81 * sin(imu_pitch);
  float acc_y = msg->linear_acceleration.y - 9.81 * cos(imu_pitch) * sin(imu_roll);
  float acc_z = msg->linear_acceleration.z - 9.81 * cos(imu_pitch) * cos(imu_roll);

  // imu队列里面装100条数据
  imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_queue_len_;

  if ((imu_ptr_last_ + 1) % imu_queue_len_ == imu_ptr_front_) {
    imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
  }

  // 维护几个队列的imu数据
  imu_time_[imu_ptr_last_] = msg->header.stamp.toSec();
  imu_roll_[imu_ptr_last_] = imu_roll;
  imu_pitch_[imu_ptr_last_] = imu_pitch;
  imu_yaw_[imu_ptr_last_] = imu_yaw;
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

  //  这里是记录imu在这个队列数据时间内的偏移
  int imu_ptr_back = (imu_ptr_last_ - 1 + imu_queue_len_) % imu_queue_len_;
  double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
  if (time_diff < 1) {
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

void XCHUSlam::gpsCB(const nav_msgs::OdometryConstPtr &msg) {
  if (!system_initialized_) {
    // 初始化
    init_pose.x = msg->pose.pose.position.x;
    init_pose.y = msg->pose.pose.position.y;
    init_pose.z = msg->pose.pose.position.z;

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z,
                                 msg->pose.pose.orientation.w)).getRPY(roll, pitch, yaw);

    init_pose.roll = roll;
    init_pose.pitch = pitch;
    init_pose.yaw = yaw;
    init_pose.updateT();

    //    pre_pose_m_ = cur_pose_m_ = pre_pose_o_ = cur_pose_o_ = diff_pose_ = Eigen::Matrix4f::Identity();
    pre_pose_m_ = cur_pose_m_ = pre_pose_o_ = cur_pose_o_ = diff_pose_ = init_pose.t;
    guess_pose = previous_pose = init_pose;
    guess_pose.updateT();
    previous_pose.updateT();

    system_initialized_ = true;
  }
  gps_deque_.push_back(*msg);
}

void XCHUSlam::visualThread() {
  ros::Rate rate(0.2);
  while (ros::ok()) {
    rate.sleep();
    publishKeyposesAndFrames();
  }
}

void XCHUSlam::transformAndSaveFinalMap() {
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr finalmap_ptr(new pcl::PointCloud<PointT>());
  // pcl::PointCloud<PointT>::Ptr map_no_ground(new pcl::PointCloud<PointT>());

  for (int i = 0; i < cloud_keyframes_.size(); ++i) {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
    *finalmap_ptr += *tmp;
    num_points += tmp->points.size();

    // 保存去除地面的点云
    //    pcl::PointCloud<PointT>::Ptr tmp1(new pcl::PointCloud<PointT>());
    //    ground_filter.convert(cloud_keyframes_[i], tmp1);
    //    tmp1 = transformPointCloud(tmp1, cloud_keyposes_6d_->points[i]);
    //    *map_no_ground += *tmp1;
    //    num_points1 += tmp1->points.size();
  }

  finalmap_ptr->width = finalmap_ptr->points.size();
  finalmap_ptr->height = 1;
  finalmap_ptr->is_dense = false;

  // 保存原图的话就不需要下采样了
  downSizeFilterGlobalMap.setInputCloud(finalmap_ptr);
  downSizeFilterGlobalMap.filter(*finalmap_ptr);

  //  map_no_ground->width = map_no_ground->points.size();
  //  map_no_ground->height = 1;
  //  map_no_ground->is_dense = false;

  // 优化后的位姿也保存一份
  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(save_dir_ + "trajectory_" + stamp + ".pcd", *poses);
  pcl::io::savePCDFile(save_dir_ + "finalCloud_" + stamp + ".pcd", *finalmap_ptr);
  // pcl::io::savePCDFile(save_dir_ + "final_no_ground_" + stamp + ".pcd", *map_no_ground);

  ROS_WARN("Save map. pose size: %d, cloud size: %d", poses->points.size(), num_points);
}

void XCHUSlam::adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time) {
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
    msg.header.frame_id = "/base_link";
    pub_undistorted_pc_.publish(msg);
  }
}

/*void XCHUSlam::extractLocalmap() {
  // 没有关键帧
  if (cloud_keyframes_.empty()) {
    ROS_WARN("No keyFrames...");
    return;
  }

  bool target_updated = false; // 是否更新
  // 关键帧数量不够的话, 加进来
  if (recent_keyframes_.size() < surround_search_num_) {
    recent_keyframes_.clear();
    // cloud_keyposes_3d_是当前的点云帧
    for (int i = cloud_keyposes_3d_->points.size() - 1; i >= 0; --i) {
      int this_key_id = int(cloud_keyposes_3d_->points[i].intensity);
      // 加进去的每一帧都做好了转换
      pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
      tf_cloud = transformPointCloud(cloud_keyframes_[this_key_id], cloud_keyposes_6d_->points[this_key_id]);
      recent_keyframes_.push_back(tf_cloud);

      if (recent_keyframes_.size() >= surround_search_num_) {
        break;
      }
    }
    target_updated = true;
  } else {

    // localmap里面帧数够了，把最老的帧pop出来
    static int latest_frame_id = cloud_keyframes_.size() - 1;
    if (latest_frame_id != cloud_keyframes_.size() - 1) {
      latest_frame_id = cloud_keyframes_.size() - 1;
      recent_keyframes_.pop_back();
      pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
      tf_cloud = transformPointCloud(cloud_keyframes_[latest_frame_id], cloud_keyposes_6d_->points[latest_frame_id]);
      recent_keyframes_.push_front(tf_cloud);
      target_updated = true;
    }
  }

  // 将附近的点云帧作为localmap匹配的target，localmap变化的时候需要更新一下target_cloud
  if (target_updated) {
    pc_target_->clear();
    for (auto keyframe : recent_keyframes_) {
      *pc_target_ += *keyframe;
    }
    // cpu_ndt_.setInputTarget(pc_target_);
    ndt_omp_->setInputTarget(pc_target_);
  }

  // 发布实时的localmap
  if (pub_localmap_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*pc_target_, *target_cloud);
    downSizeFilterLocalmap.setInputCloud(target_cloud);
    downSizeFilterLocalmap.filter(*target_cloud);
    pcl::toROSMsg(*target_cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub_localmap_.publish(msg);
  }
}*/

void XCHUSlam::updateLocalmap() {
  // 没有关键帧
  if (cloud_keyframes_.empty()) {
    ROS_ERROR("No keyFrames...");
    return;
  }

  // 更新localmap
  distance_shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (distance_shift >= min_add_scan_shift) {
    localmap_size += distance_shift;
    odom_size += distance_shift;

    pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*pc_source_, *target_cloud, current_pose.t);

    // 只有在fitnessscore状态好的情况下才选取作为关键帧加入到localmap中
    //    if(fitness_score< 0.2 && final_num_iteration < 4)
    //    {       }
    localmap += *target_cloud;
    submap += *target_cloud;
    added_pose = current_pose;

    // 更新lcaomap
    ndt_omp_->setInputTarget(localmap_ptr);
  }

  // 发布实时的localmap
  if (pub_localmap_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr localmap_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(localmap, *localmap_msg_ptr);
    localmap_msg_ptr->header.frame_id = "map";
    pub_localmap_.publish(localmap_msg_ptr);
  }

  // 检查localmap
  if (localmap_size >= max_localmap_size) {
    ROS_WARN("Clear local cloud...");
    localmap = submap;
    submap.clear();
    localmap_size = 0.0;
  }
}

bool XCHUSlam::saveKeyframesAndFactor() {

  Eigen::Quaternionf current_q(current_pose.t.block<3, 3>(0, 0));

  if (cloud_keyposes_3d_->points.empty()) {
    // 第一帧进来的时候直接添加PriorFactor
    gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch, current_pose.yaw),
                                                Point3(current_pose.x, current_pose.y, current_pose.z)), prior_noise_));

    initial_estimate_.insert(0, Pose3(Rot3::Quaternion(current_q.w(), current_q.x(), current_q.y(), current_q.z()),
                                      Point3(current_pose.x, current_pose.y, current_pose.z)));
    // 更新current_pose
    pre_keypose_ = cur_pose_ndt_;
  } else {

    // 添加GPS factor
    if (use_gps_) {
      addGPSFactor();
    }

    // 上一个关键帧位姿
    const auto &pre_pose = cloud_keyposes_6d_->points[cloud_keyposes_3d_->points.size() - 1];

    // 关集帧之间的距离大于0.5米才将其加到位姿图中
    if (std::pow(current_pose.x - pre_pose.x, 2) +
        std::pow(current_pose.y - pre_pose.y, 2) +
        std::pow(current_pose.z - pre_pose.z, 2) <
        keyframe_dist_ * keyframe_dist_) {
//    if (distance_shift < keyframe_dist_) {
      ROS_WARN("frames are too close...");
      return false;
    }

    // 添加相邻帧之间的约束关系BetweenFactor
    gtsam::Pose3 pose_from =
        Pose3(Rot3::RzRyRx(pre_pose.roll, pre_pose.pitch, pre_pose.yaw), Point3(pre_pose.x, pre_pose.y, pre_pose.z));
    gtsam::Pose3 pose_to =
        Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch * 0, current_pose.yaw),
              Point3(current_pose.x, current_pose.y, current_pose.z * 0));

    gtSAMgraph_.add(BetweenFactor<Pose3>(cloud_keyposes_3d_->points.size() - 1, cloud_keyposes_3d_->points.size(),
                                         pose_from.between(pose_to), odom_noise_));
    initial_estimate_.insert(cloud_keyposes_3d_->points.size(),
                             Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch * 0, current_pose.yaw),
                                   Point3(current_pose.x, current_pose.y, current_pose.z * 0)));

  }

  // 增量平滑之后更新current_pose
  isam->update(gtSAMgraph_, initial_estimate_);
  isam->update();

  // update multiple-times till converge
  if (loop_closed_) {
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();
  }
  gtSAMgraph_.resize(0);
  initial_estimate_.clear();

  // 保存关键帧和位姿, 更新current_pose
  PointT this_pose_3d;
  PointXYZIRPYT this_pose_6d;
  Pose3 latest_estimate;
  isam_current_estimate_ = isam->calculateEstimate();
  latest_estimate = isam_current_estimate_.at<Pose3>(isam_current_estimate_.size() - 1);
  // 边缘化得到每个变量的协方差
  poseCovariance = isam->marginalCovariance(isam_current_estimate_.size() - 1);

  // intensity字段表示当前关键帧的序号
  current_pose.x = this_pose_6d.x = this_pose_3d.x = latest_estimate.translation().x();
  current_pose.y = this_pose_6d.y = this_pose_3d.y = latest_estimate.translation().y();
  current_pose.z = this_pose_6d.z = this_pose_3d.z = latest_estimate.translation().z();
  current_pose.roll = this_pose_6d.roll = latest_estimate.rotation().roll();
  current_pose.pitch = this_pose_6d.pitch = latest_estimate.rotation().pitch();
  current_pose.yaw = this_pose_6d.yaw = latest_estimate.rotation().yaw();
  //this_pose_6d.time = cur_pose_ndt_.header.stamp.toSec();
  this_pose_6d.intensity = this_pose_3d.intensity = cloud_keyposes_3d_->points.size();
  this_pose_6d.time = current_scan_time.toSec();
  current_pose.updateT();

  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyposes_6d_->points.push_back(this_pose_6d);

  // 更新 pre_keypose_
  if (cloud_keyposes_3d_->points.size() > 1) {
    pre_keypose_.pose.pose.position.x = this_pose_3d.x;
    pre_keypose_.pose.pose.position.y = this_pose_3d.y;
    pre_keypose_.pose.pose.position.z = this_pose_3d.z;
    pre_keypose_.pose.pose.orientation.w = latest_estimate.rotation().toQuaternion().w();
    pre_keypose_.pose.pose.orientation.x = latest_estimate.rotation().toQuaternion().x();
    pre_keypose_.pose.pose.orientation.y = latest_estimate.rotation().toQuaternion().y();
    pre_keypose_.pose.pose.orientation.z = latest_estimate.rotation().toQuaternion().z();
    pre_keypose_.header.stamp = current_scan_time;
  }

  // 更新当前pose
  cur_pose_m_ = current_pose.t;

  // 存储关键帧
  pcl::PointCloud<PointT>::Ptr cur_keyframe(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*pc_source_, *cur_keyframe);
  for (auto &p : cur_keyframe->points) {
    p.intensity = this_pose_3d.intensity;
  }
  cloud_keyframes_.push_back(cur_keyframe);

  // pc_source
  //  pcl::PointCloud<PointT>::Ptr pc_m(new pcl::PointCloud<PointT>());
  //  pcl::transformPointCloud(*pc_source_, *pc_m, cur_pose_m_);
  // cpu_ndt_.updateVoxelGrid(pc_m);

  // 根据current和previous两帧之间的scantime,以及两帧之间的位置,计算两帧之间的变化量
  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  diff_pose.x = current_pose.x - previous_pose.x;
  diff_pose.y = current_pose.y - previous_pose.y;
  diff_pose.z = current_pose.z - previous_pose.z;
  diff_pose.yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  double diff = std::sqrt(diff_pose.x * diff_pose.x + diff_pose.y * diff_pose.y + diff_pose.z * diff_pose.z);

  // 更新速度, 修正imu估计的速度
  current_velocity_x = diff_pose.x / secs;
  current_velocity_y = diff_pose.y / secs;
  current_velocity_z = diff_pose.z / secs;

  current_velocity_imu_x = current_velocity_x;  // 修正imu速度
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // 统一所有的pose
  current_pose_imu_odom = current_pose_odom = current_pose_imu = current_pose;

  // Update position and posture. current_pos -> previous_pos
  pre_pose_m_ = cur_pose_m_;
  pre_pose_ndt_ = cur_pose_ndt_;

  previous_pose = current_pose;
  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  diff_imu_pose.init();
  diff_imu_odom_pose.init();
  diff_odom_pose.init();

  //ROS_INFO("saveKeyframesAndFactor: %d points", cur_keyframe->points.size());

  return true;
}

void XCHUSlam::publishKeyposesAndFrames() {
  // 发布关键帧位姿
  if (pub_keyposes_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_keyposes_.publish(msg);
  }
  // 发布全局地图，非最终地图，只是里程计关键帧拼接而成的
  if (pub_globalmap_.getNumSubscribers() > 0) {
    int num_points = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<PointT>::Ptr globalmap_ptr(new pcl::PointCloud<PointT>());
    Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
    // 把关键帧拼接而成，这里关键帧及其对应的位姿都存储好了
    for (int i = 0; i < cloud_keyframes_.size(); ++i) {
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
      tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
      *globalmap_ptr += *tmp;
      num_points += tmp->points.size();
    }

    // downsample visualized points
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalmap_ptr);
    downSizeFilterGlobalMapKeyFrames.filter(*globalmap_ptr);

    pcl::toROSMsg(*globalmap_ptr, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_globalmap_.publish(msg);
    ROS_INFO("Map Size: %d points", num_points);
  }
}

void XCHUSlam::processCloud(const pcl::PointCloud<PointT> &tmp, const ros::Time &scan_time) {
  auto start = std::chrono::system_clock::now();
  current_scan_time = scan_time;

  pc_source_->clear();
  pc_source_.reset(new pcl::PointCloud<PointT>(tmp));
  downSizeFilterSource.setInputCloud(pc_source_);
  downSizeFilterSource.filter(*pc_source_);

  // 第一帧点云进来, 直接存为target, 初始化起始位置
  if (cloud_keyframes_.empty()) {
    ROS_INFO("Init target map and pose...");
    pcl::PointCloud<PointT>::Ptr first_cloud(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*pc_source_, *first_cloud, guess_pose.t * tf_b2l_);  // tf_btol为初始变换矩阵

    *pc_target_ += *first_cloud;
    localmap += *first_cloud;
    localmap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(localmap));

    // cpu_ndt_.setInputTarget(pc_target_);
    // ndt_omp_->setInputTarget(pc_target_);


    ndt_omp_->setInputTarget(localmap_ptr);
  } else {
    localmap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(localmap));
  }
  downSizeFilterLocalmap.setInputCloud(localmap_ptr);
  downSizeFilterLocalmap.filter(*localmap_ptr);

  // 计算初始姿态，上一帧点云结果+传感器畸变
  guess_pose.x = previous_pose.x + diff_pose.x;  // 初始时diff_x等都为0
  guess_pose.y = previous_pose.y + diff_pose.y;
  guess_pose.z = previous_pose.z + diff_pose.z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_pose.yaw;
  guess_pose.updateT();

  // 根据是否使用imu和odom, 按照不同方式更新guess_pose(xyz,or/and rpy)
  if (use_imu_ && use_odom_)
    imuOdomCalc(current_scan_time);
  if (use_imu_ && !use_odom_)
    imuCalc(current_scan_time);
  if (!use_imu_ && use_odom_)
    odomCalc(current_scan_time);

  // 只是为了把上面不同方式的guess_pose都标准化成guess_pose_for_ndt,为了后续操作方便
  POSE guess_pose_for_ndt;
  if (use_imu_ && use_odom_)
    guess_pose_for_ndt = guess_pose_imu_odom;
  else if (use_imu_ && !use_odom_)
    guess_pose_for_ndt = guess_pose_imu;
  else if (!use_imu_ && use_odom_)
    guess_pose_for_ndt = guess_pose_odom;
  else
    guess_pose_for_ndt = guess_pose;
  guess_pose_for_ndt.updateT();

  Eigen::Matrix4f init_guess = guess_pose_for_ndt.t * tf_b2l_;
  pre_pose_m_ = init_guess;

  // 发布预测的odom
  Eigen::Quaternionf guess_q(pre_pose_m_.block<3, 3>(0, 0));
  nav_msgs::Odometry predict_msg;
  predict_msg.header.stamp = current_scan_time;
  predict_msg.header.frame_id = "map";
  predict_msg.pose.pose.position.x = pre_pose_m_(0, 3);
  predict_msg.pose.pose.position.y = pre_pose_m_(1, 3);
  predict_msg.pose.pose.position.z = pre_pose_m_(2, 3);
  predict_msg.pose.pose.orientation.w = guess_q.w();
  predict_msg.pose.pose.orientation.x = guess_q.x();
  predict_msg.pose.pose.orientation.y = guess_q.y();
  predict_msg.pose.pose.orientation.z = guess_q.z();
  pub_predict_odom_.publish(predict_msg);

  pcl::PointCloud<PointT>::Ptr aligned_cloud_(new pcl::PointCloud<PointT>);
  ndt_omp_->setInputSource(pc_source_);
  ndt_omp_->align(*aligned_cloud_, init_guess);
  fitness_score_ = ndt_omp_->getFitnessScore();
  final_transformation_ = ndt_omp_->getFinalTransformation();
  has_converged_ = ndt_omp_->hasConverged();
  final_iters_ = ndt_omp_->getFinalNumIteration();

  // bask_link 需要排除掉全局起始点偏移造成的影响，全局起点偏移就是雷达起始有个高度和yaw偏角
  // t_localizer是相对位姿, t_base_link对应的是全局位姿
  t_base_link = final_transformation_ * tf_b2l_.inverse();
  cur_pose_m_ = t_base_link;

  Eigen::Quaternionf tmp_q(t_base_link.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  // 更新cur_pose_ndt_(msg)和cur_pose_m_(t)
  cur_pose_ndt_.pose.pose.position.x = t_base_link(0, 3);
  cur_pose_ndt_.pose.pose.position.y = t_base_link(1, 3);
  cur_pose_ndt_.pose.pose.position.z = t_base_link(2, 3);
  cur_pose_ndt_.pose.pose.orientation.w = tmp_q.w();
  cur_pose_ndt_.pose.pose.orientation.x = tmp_q.x();
  cur_pose_ndt_.pose.pose.orientation.y = tmp_q.y();
  cur_pose_ndt_.pose.pose.orientation.z = tmp_q.z();
  cur_pose_ndt_.header.stamp = current_scan_time;

  // Update ndt_pose.  //  current_pose对应的是全局下的坐标!
  current_pose.x = t_base_link(0, 3);
  current_pose.y = t_base_link(1, 3);
  current_pose.z = t_base_link(2, 3);
  current_pose.roll = roll;
  current_pose.pitch = pitch;
  current_pose.yaw = yaw;
  current_pose.t = t_base_link;

  // 以下:对current_pose做tf变换
  tf::Quaternion q;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  // 发布里程计位姿
  if (pub_final_odom_.getNumSubscribers() > 0) {
    nav_msgs::Odometry updated_msg;
    updated_msg.header.stamp = current_scan_time;
    updated_msg.header.frame_id = "map";
    updated_msg.pose = cur_pose_ndt_.pose;
    //    updated_msg.pose.covariance[0] = poseCovariance(0,0);
    //    updated_msg.pose.covariance[7] = poseCovariance(1,1);
    //    updated_msg.pose.covariance[14] = poseCovariance(2,2);
    pub_final_odom_.publish(updated_msg);
  }
  // 发布实时的点云帧
  if (pub_current_frames_.getNumSubscribers() > 0) {
    aligned_cloud_->clear();
    pcl::transformPointCloud(*pc_source_, *aligned_cloud_, final_transformation_);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*aligned_cloud_, msg);
    msg.header.frame_id = "map";
    pub_current_frames_.publish(msg);
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  std::cout << "current cloud size: " << pc_source_->size() << " points." << std::endl;
  std::cout << "target cloud size: " << localmap_ptr->points.size() << " points." << std::endl;
  std::cout << "iteration: " << final_iters_ << std::endl;
  std::cout << "fitness score: " << fitness_score_ << std::endl;
  std::cout << "transformation matrix:" << std::endl;
  std::cout << final_transformation_ << std::endl;
  std::cout << "used time:" << elapsed.count() << std::endl;
  std::cout << "shift: " << distance_shift << std::endl;
}

void XCHUSlam::loopClosureThread() {
  ros::Duration duration(1);
  while (ros::ok()) {
    performLoopClosure();
    duration.sleep();
  }
}

void XCHUSlam::performLoopClosure() {
  // 保证有关键帧
  if (cloud_keyposes_3d_->points.empty()) {
    ROS_ERROR("No key pose, can not get clouser..");
    return;
  }
  // 持续检测回环,未检测到就退出
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

  // 注意input是当前帧
  icp.setInputSource(latest_keyframe_);   // 已经转换到map上的当前帧点云
  icp.setInputTarget(near_history_keyframes_); // 已经拼好的回环帧的localmap
  pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());

  //  初始位姿哪里来，就是当前的位姿
  Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
  initial_guess.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].roll,
                            Eigen::Vector3f::UnitX())).toRotationMatrix();
  initial_guess(0, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].x;
  initial_guess(1, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].y;
  initial_guess(2, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].z;
  icp.align(*unused_result);

  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();
  Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
  Eigen::Quaternionf tmp_q(icp_trans.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  if (has_converged == false || fitness_score > history_fitness_score_) {
    ROS_ERROR("loop cannot closed score %f ", fitness_score);
    return;
  } else {
    ROS_WARN("loop closed  score %f ", fitness_score);
    // 回环检测关键帧
    //    if (pub_icp_keyframes_.getNumSubscribers() > 0) {
    //      pcl::PointCloud<PointT>::Ptr closed_cloud(new pcl::PointCloud<PointT>());
    //      pcl::transformPointCloud(*latest_keyframe_, *closed_cloud, correction_frame);
    //      sensor_msgs::PointCloud2 msg;
    //      pcl::toROSMsg(*closed_cloud, msg);
    //      msg.header.stamp.fromSec(cloud_keyposes_6d_->points[latest_history_frame_id_].time);
    //      msg.header.frame_id = "map";
    //      pub_icp_keyframes_.publish(msg);
    //    }
  }

  Eigen::Matrix4f t_wrong = initial_guess;
  Eigen::Matrix4f T_correct = icp_trans * t_wrong;
  Eigen::Quaternionf R_correct(T_correct.block<3, 3>(0, 0));

  // 更新因子图
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(R_correct.w(), R_correct.x(), R_correct.y(), R_correct.z()),
                                 Point3(T_correct(0, 3), T_correct(1, 3), T_correct(2, 3)));
  // closest_history_frame_id_是找到的回环帧id
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

  // 添加约束边
  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph_.add(BetweenFactor<Pose3>(latest_history_frame_id_, closest_history_frame_id_, pose_from.between(pose_to),
                                       constraint_noise_));
  isam->update(gtSAMgraph_);
  isam->update();
  isam->update();
  isam->update();
  isam->update();
  isam->update();
  gtSAMgraph_.resize(0);

  std::cout << "------------------------------LOOP CLOSE!!!-----------------------------------" << std::endl;
  std::cout << "Loop Fitness score: " << fitness_score << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << T_correct << std::endl;
  std::cout << "loop shift: " << std::sqrt(std::pow(icp_trans(0, 3) - initial_guess(0, 3), 2) +
      std::pow(icp_trans(1, 3) - initial_guess(1, 3), 2) +
      std::pow(icp_trans(2, 3) - initial_guess(2, 3), 2)) << std::endl;
  std::cout << "------------------------------LOOP CLOSE!!!-----------------------------------" << std::endl;

  loop_closed_ = true;
}

bool XCHUSlam::detectLoopClosure() {

  // near_history_keyframes_是附近的关键帧
  latest_keyframe_->clear();
  near_history_keyframes_->clear();

  std::lock_guard<std::mutex> lock(mtx_);
  PointT cur_pose;
  cur_pose.x = cur_pose_m_(0, 3);
  cur_pose.y = cur_pose_m_(1, 3);
  cur_pose.z = cur_pose_m_(2, 3);
  std::vector<int> search_idx_; // 搜索到紧邻点的id
  std::vector<float> search_dist_; // 对应的距离平方
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  kdtree_poses_->radiusSearch(cur_pose, history_search_radius_, search_idx_, search_dist_);

  // kdtree找到距离当前位置最近的历史关键帧
  latest_history_frame_id_ = cloud_keyframes_.size() - 1;
  closest_history_frame_id_ = -1;
  for (int i = 0; i < search_idx_.size(); ++i) {
    // 历史帧与当前帧必须间隔时间足够远
    if (cur_pose_ndt_.header.stamp.toSec() - cloud_keyposes_6d_->points[search_idx_[i]].time > 20) {
      closest_history_frame_id_ = search_idx_[i];
      break;
    }
  }
  if (closest_history_frame_id_ == -1) {
    return false;
  }
  std::cout << "find loopclosure: " << latest_history_frame_id_ << " and " << closest_history_frame_id_ << std::endl;

  // 复制一份找到的回环帧
  pcl::PointCloud<PointT>::Ptr tmp_ptr(new pcl::PointCloud<PointT>());
  tmp_ptr = transformPointCloud(cloud_keyframes_[latest_history_frame_id_],
                                cloud_keyposes_6d_->points[latest_history_frame_id_]);
  pcl::copyPointCloud(*tmp_ptr, *latest_keyframe_);


  // 把回环帧附近前后20帧拼接成localmap
  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i) {
    j = closest_history_frame_id_ + i;
    if (j < 0 || j >= latest_history_frame_id_) {
      continue;
    }
    *tmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
  }

  downSizeFilterHistoryKeyframes.setInputCloud(tmp_cloud);
  downSizeFilterHistoryKeyframes.filter(*near_history_keyframes_);

  return true;
}

void XCHUSlam::correctPoses() {
  // 检测到回环了在下一帧中更新全局位姿 cloud_keyposes_6d_和cloud_keyposes_3d_
  if (loop_closed_) {
    //    recent_keyframes_.clear();
    localmap_ptr->clear();
    ROS_WARN("corret loop pose...");
    int num_poses = isam_current_estimate_.size();
    for (int i = 0; i < num_poses; ++i) {
      cloud_keyposes_6d_->points[i].x = cloud_keyposes_3d_->points[i].x =
          isam_current_estimate_.at<Pose3>(i).translation().x();
      cloud_keyposes_6d_->points[i].y = cloud_keyposes_3d_->points[i].y =
          isam_current_estimate_.at<Pose3>(i).translation().y();
      cloud_keyposes_6d_->points[i].z = cloud_keyposes_3d_->points[i].z =
          isam_current_estimate_.at<Pose3>(i).translation().z();
      cloud_keyposes_6d_->points[i].roll = isam_current_estimate_.at<Pose3>(i).rotation().roll();
      cloud_keyposes_6d_->points[i].pitch = isam_current_estimate_.at<Pose3>(i).rotation().pitch();
      cloud_keyposes_6d_->points[i].yaw = isam_current_estimate_.at<Pose3>(i).rotation().yaw();
    }
    loop_closed_ = false;
  }
}

void XCHUSlam::imuOdomCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();  // static声明的变量只会在第一次使用时被声明,因此不会被覆盖

  // imu信息处理,计算 -- imu只使用陀螺仪,即 只输出转角信息roll,pitch,yaw
  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;  // 更新current_pose_imu_odom相关,作为历史记录
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  // odom信息处理,计算 -- xyz移动距离的计算,融合odom的速度(位移)信息和imu的转角信息
  double diff_distance = odom.twist.twist.linear.x * diff_time;
  diff_imu_odom_pose.x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  diff_imu_odom_pose.y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  diff_imu_odom_pose.z += diff_distance * sin(-current_pose_imu_odom.pitch);

  diff_imu_odom_pose.roll += diff_imu_roll;
  diff_imu_odom_pose.pitch += diff_imu_pitch;
  diff_imu_odom_pose.yaw += diff_imu_yaw;

  // ==> 最终的目的是融合imu和odom输出一个guess_pose
  // 注:guess_pose是在previous_pose基础上叠加一个offset,包括xyz的和rpy的
  // xyz的offset需要融合imu的转角和odom的速度(位移)
  // rpy的offset直接采用imu的rpy偏差值
  guess_pose_imu_odom.x = previous_pose.x + diff_imu_odom_pose.x;
  guess_pose_imu_odom.y = previous_pose.y + diff_imu_odom_pose.y;
  guess_pose_imu_odom.z = previous_pose.z + diff_imu_odom_pose.z;
  guess_pose_imu_odom.roll = previous_pose.roll + diff_imu_odom_pose.roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + diff_imu_odom_pose.pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + diff_imu_odom_pose.yaw;

  previous_time = current_time;
}

void XCHUSlam::imuCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  // 对imu由于不平衡造成的补偿问题,在这里解决
  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
      std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
      std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
  diff_imu_pose.x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  diff_imu_pose.y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  diff_imu_pose.z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;  // imu的速度值会通过slam进行修正,以避免累计误差
  current_velocity_imu_y += accY * diff_time;  // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
  current_velocity_imu_z += accZ * diff_time;    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

  diff_imu_pose.roll += diff_imu_roll;
  diff_imu_pose.pitch += diff_imu_pitch;
  diff_imu_pose.yaw += diff_imu_yaw;

  guess_pose_imu.x = previous_pose.x + diff_imu_pose.x;
  guess_pose_imu.y = previous_pose.y + diff_imu_pose.y;
  guess_pose_imu.z = previous_pose.z + diff_imu_pose.z;
  guess_pose_imu.roll = previous_pose.roll + diff_imu_pose.roll;
  guess_pose_imu.pitch = previous_pose.pitch + diff_imu_pose.pitch;
  guess_pose_imu.yaw = previous_pose.yaw + diff_imu_pose.yaw;

  previous_time = current_time;
}

void XCHUSlam::odomCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  diff_odom_pose.x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  diff_odom_pose.y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  diff_odom_pose.z += diff_distance * sin(-current_pose_odom.pitch);

  diff_odom_pose.roll += diff_odom_roll;
  diff_odom_pose.pitch += diff_odom_pitch;
  diff_odom_pose.yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose.x + diff_odom_pose.x;
  guess_pose_odom.y = previous_pose.y + diff_odom_pose.y;
  guess_pose_odom.z = previous_pose.z + diff_odom_pose.z;
  guess_pose_odom.roll = previous_pose.roll + diff_odom_pose.roll;
  guess_pose_odom.pitch = previous_pose.pitch + diff_odom_pose.pitch;
  guess_pose_odom.yaw = previous_pose.yaw + diff_odom_pose.yaw;

  previous_time = current_time;
}

void XCHUSlam::imuUpSideDown(sensor_msgs::Imu &input) {
  double input_roll, input_pitch, input_yaw;
  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input.orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input.angular_velocity.x *= -1;
  input.angular_velocity.y *= -1;
  input.angular_velocity.z *= -1;

  input.linear_acceleration.x *= -1;
  input.linear_acceleration.y *= -1;
  input.linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

  //  input_yaw += M_PI/2;
  input.orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

void XCHUSlam::addGPSFactor() {
  if (gps_deque_.empty())
    return;

  // wait for system initialized and settles down
  if (cloud_keyposes_3d_->points.empty())
    return;
  else {
    // 开始5米不使用GPS
    if (pointDistance(cloud_keyposes_3d_->front(), cloud_keyposes_3d_->back()) < 5.0)
      return;
  }

  // pose covariance small, no need to correct
  std::cout << "cov: " << poseCovariance(3, 3) << ", " << poseCovariance(4, 4) << ", " << poseCovariance.size()
            << std::endl;
  if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
    return;

  ROS_WARN("ADD gps factor, gps deque size %d ...", gps_deque_.size());

  // pose的协方差比较大的时候才去添加gps factor
//  if (!gps_deque_.empty()) {
  // 时间戳对齐
/*  ros::Time gps_time = gps_deque_.front().header.stamp;

  nav_msgs::Odometry odom_msg;
  bool gps_type = false;
  auto odom_iter = gps_deque_.begin();
  for (odom_iter; odom_iter != gps_deque_.end(); odom_iter++) {
    if (current_scan_time < (*odom_iter).header.stamp) {
      break;
    }
    odom_msg.header.stamp = (*odom_iter).header.stamp;
    odom_msg.pose = (*odom_iter).pose;
    gps_type = true;
  }
  gps_deque_.erase(gps_deque_.begin(), odom_iter);

  if (!gps_type) {
    ROS_ERROR("gps lidar time aligned false");
    return;
  }

  double off_time = current_scan_time.toSec() - odom_msg.header.stamp.toSec();
  ROS_WARN(" gps lidar off time: %f ", off_time);

  // GPS too noisy, skip
  float noise_x = odom_msg.pose.covariance[0];
  float noise_y = odom_msg.pose.covariance[7];
  float noise_z = odom_msg.pose.covariance[14];
  if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
    return;

  float gps_x = odom_msg.pose.pose.position.x;
  float gps_y = odom_msg.pose.pose.position.y;
  float gps_z = odom_msg.pose.pose.position.z;
  //ROS_INFO("GPS ENU XYZ : %f, %f, %f", gps_x, gps_y, gps_z);

  if (!useGpsElevation) {
    gps_z = current_pose.z;  // gps的z一般不可信
    noise_z = 0.01;
  }

  // GPS not properly initialized (0,0,0)
  if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
    return;

  // 添加GPS因子
  gtsam::Vector Vector3(3);
  Vector3 << noise_x, noise_y, noise_z;
  noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3); // 噪声定义
  gtsam::GPSFactor gps_factor(cloud_keyposes_3d_->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
  gtSAMgraph_.add(gps_factor);

  loop_closed_ = true;*/

  // pose的协方差比较大的时候才去添加gps factor
  while (!gps_deque_.empty()) {

    // 时间戳对齐
    if (gps_deque_.front().header.stamp.toSec() < current_scan_time.toSec() - 0.1) {
      // message too old
      gps_deque_.pop_front();
    } else if (gps_deque_.front().header.stamp.toSec() > current_scan_time.toSec() + 0.1) {
      // message too new
      break;
    } else {
      nav_msgs::Odometry thisGPS = gps_deque_.front();
      gps_deque_.pop_front();


      double off_time = current_scan_time.toSec() - thisGPS.header.stamp.toSec();
      ROS_WARN(" gps lidar off time: %f ", off_time);

      // GPS too noisy, skip
      float noise_x = thisGPS.pose.covariance[0];
      float noise_y = thisGPS.pose.covariance[7];
      float noise_z = thisGPS.pose.covariance[14];
      if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
        continue;

      float gps_x = thisGPS.pose.pose.position.x;
      float gps_y = thisGPS.pose.pose.position.y;
      float gps_z = thisGPS.pose.pose.position.z;
      if (!useGpsElevation) {
        gps_z = current_pose.z;  // gps的z一般不可信
        noise_z = 0.01;
      }

      // GPS not properly initialized (0,0,0)
      if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
        continue;

      // 添加GPS因子
      gtsam::Vector Vector3(3);
      Vector3 << noise_x, noise_y, noise_z;
      noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3); // 噪声定义
      gtsam::GPSFactor gps_factor(cloud_keyposes_3d_->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
      gtSAMgraph_.add(gps_factor);

      loop_closed_ = true;
      break;
    }
  }

}

/*while (!gps_deque_.empty()) {
  // 时间戳对齐
  double gps_time = gps_deque_.front().header.stamp.toSec();
  double off_time = current_scan_time.toSec() - gps_time;
  ROS_WARN("off set time: %f ", off_time);
  // 根据自己的gps频率设定，我这里是1hz
  if (2.0 < current_scan_time.toSec() - gps_time) {
    // message too old
    gps_deque_.pop_front();
  } else if (gps_time > current_scan_time.toSec() + 2.0) {
    // message too new
    break;
  } else {

    nav_msgs::Odometry odom_msg = gps_deque_.front();
    gps_deque_.pop_front();

    // GPS too noisy, skip
    float noise_x = odom_msg.pose.covariance[0];
    float noise_y = odom_msg.pose.covariance[7];
    float noise_z = odom_msg.pose.covariance[14];
    if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
      continue;

    float gps_x = odom_msg.pose.pose.position.x;
    float gps_y = odom_msg.pose.pose.position.y;
    float gps_z = odom_msg.pose.pose.position.z;
    ROS_INFO("GPS ENU XYZ : %f, %f, %f", gps_x, gps_y, gps_z);

    if (!useGpsElevation) {
      gps_z = current_pose.z;  // gps的z一般不可信
      noise_z = 0.01;
    }

    // GPS not properly initialized (0,0,0)
    if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
      continue;

    // 添加GPS因子
    gtsam::Vector Vector3(3);
    Vector3 << noise_x, noise_y, noise_z;
    noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3); // 噪声定义
    gtsam::GPSFactor gps_factor(cloud_keyposes_3d_->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
    gtSAMgraph_.add(gps_factor);

    loop_closed_ = true;
    break;
  }
}*/



