/**
 * @file xchu_mapping.cpp
 * @author XCHU (2022087641@qq.com)
 * @brief
 * @version 1.0
 * @date 2020-09-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <xchu_mapping/odom_node.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "xchu_mapping_node");
  ROS_INFO("\033[1;32m---->\033[0m XCHU Odometry Started.");

  LidarOdom mapping;
  ros::Rate rate(20);
  while (ros::ok()) {
    mapping.Run();
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
  return 0;
}

LidarOdom::LidarOdom() : nh("~") {
// 初始化参数
  ParamInitial();

  current_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_points", 10);
  current_odom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_odom", 100);

  points_sub = nh.subscribe("/filtered_points", 10, &LidarOdom::PcCB, this);
  odom_sub = nh.subscribe("/odom_raw", 50, &LidarOdom::OdomCB, this);// 编码器
  imu_sub = nh.subscribe("/kitti/oxts/imu", 500, &LidarOdom::ImuCB, this);
}

void LidarOdom::ParamInitial() {
  nh.param<float>("ndt_resolution", ndt_res, 2.0);
  nh.param<double>("ndt_step_size", step_size, 0.1);
  nh.param<double>("ndt_trans_eps", trans_eps, 0.01);
  nh.param<int>("ndt_max_iter", max_iter, 50);
  nh.param<double>("min_add_scan_shift", min_add_scan_shift, 0.5);
  nh.param<double>("max_submap_size", max_localmap_size, 15);
  nh.param<std::string>("map_saved_dir", map_saved_dir, "");
  nh.param<bool>("use_imu", _use_imu, true);
  nh.param<bool>("use_odom", _use_odom, false);
  nh.param<bool>("imu_upside_down", _imu_upside_down, false);
  nh.param<bool>("incremental_voxel_update", _incremental_voxel_update, true);

  nh.param<int>("ndt_method_type", method_type_temp, 3);
  _method_type = static_cast<MethodType>(method_type_temp);
  if (_method_type == MethodType::use_pcl) {
    std::cout << ">> Use PCL NDT <<" << std::endl;
    pcl_ndt.setTransformationEpsilon(trans_eps);
    pcl_ndt.setStepSize(step_size);
    pcl_ndt.setResolution(ndt_res);
    pcl_ndt.setMaximumIterations(max_iter);
  } else if (_method_type == MethodType::use_cpu) {
    std::cout << ">> Use CPU NDT <<" << std::endl;
    cpu_ndt.setTransformationEpsilon(trans_eps);
    cpu_ndt.setStepSize(step_size);
    cpu_ndt.setResolution(ndt_res);
    cpu_ndt.setMaximumIterations(max_iter);
  } else if (_method_type == MethodType::use_omp) {
    std::cout << ">> Use OMP NDT <<" << std::endl;
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(omp_get_num_threads());  //  设置最大线程, 注意：需要引入头文件omp.h
    ndt->setTransformationEpsilon(trans_eps);
    ndt->setStepSize(step_size);
    ndt->setResolution(ndt_res);
    ndt->setMaximumIterations(max_iter);
    // 注意：此处设置ndt参数之后才能赋值给registration,否则后面无法使用getFinalScore函数！
    omp_ndt = ndt;
  } else {
    ROS_ERROR("Please Define _method_type to conduct NDT");
  }

  // 这里一般需要设定初始的雷达高度init_z
  nh.param<double>("init_x", _tf_x, 0);
  nh.param<double>("init_y", _tf_y, 0);
  nh.param<double>("init_z", _tf_z, 0);
  nh.param<double>("init_roll", _tf_roll, 0);
  nh.param<double>("init_pitch", _tf_pitch, 0);
  nh.param<double>("init_yaw", _tf_yaw, 0);
  // 利用旋转向量求变换矩阵
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  // 初始化tf_btol == 根据初始化得到的tl_btol等初始化tf_btol
  // 以获得初始变换矩阵 ---当车辆初始的起点不在预定义的globalMap原点时,就需要tf_btol了
  tf_b2l = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix(); // 车身到雷达的变换
  tf_l2b = tf_b2l.inverse();  // 将tf_btol取逆 --因为我们后面配准的时候都是相对于localMap的原点的,因此tf_ltob.inv将作为补偿的矩阵

  downSizeFilterKeyFrames.setLeafSize(0.5, 0.5, 0.5); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterLocalmap.setLeafSize(1.0, 1.0, 1.0);
  downSizeFilterGlobalMap.setLeafSize(0.5, 0.5, 0.5);

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointTypePose>());

  scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  // filtered_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  transformed_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());

  gnss_transform.setIdentity();  // 保存GPS信号的变量
  t_localizer.setIdentity();
  t_base_link.setIdentity();

  // 定义各种差异值(两次采集数据之间的差异,包括点云位置差异,imu差异,odom差异,imu-odom差异)
  diff = 0.0;
  //diff_x = diff_y = diff_z = 0.0;  // current_pose - previous_pose // 定义两帧点云差异值 --以确定是否更新点云等

  // 定义速度值 --包括实际速度值,和imu取到的速度值
  current_velocity_x = current_velocity_y = current_velocity_z = 0.0;
  current_velocity_imu_x = current_velocity_imu_y = current_velocity_imu_z = 0.0;
  // diff_x = diff_y = diff_z = diff_yaw = 0.0;
//  offset_imu_x = offset_imu_y = offset_imu_z = offset_imu_roll = offset_imu_pitch = offset_imu_yaw = 0.0;
//  offset_odom_x = offset_odom_y = offset_odom_z = offset_odom_roll = offset_odom_pitch = offset_odom_yaw = 0.0;
//  offset_imu_odom_x = offset_imu_odom_y = offset_imu_odom_z = offset_imu_odom_roll = offset_imu_odom_pitch =
//  offset_imu_odom_yaw = 0.0;
}

void LidarOdom::Run() {
//  while (ros::ok()) {
  if (_use_imu) {
    //ROS_WARN("USE IMU!");
    // std::cout << "use imu" << std::endl;
    if (!cloud_queue_.empty() && !imu_queue_.empty()) {
      //align time stamp
      mutex_lock.lock();
      double time_diff = cloud_queue_.front()->header.stamp.toSec() - imu_queue_.front()->header.stamp.toSec();
      ROS_WARN("IMU CLOUD TIME DIFF %f", time_diff);
      if (!imu_queue_.empty() && imu_queue_.front()->header.stamp.toSec()
          < cloud_queue_.front()->header.stamp.toSec() - 0.5 * 0.1) {
        ROS_WARN(
            "odom_node: time stamp unaligned error and imu discarded, pls check your data; odom time %f, pc time %f",
            imu_queue_.front()->header.stamp.toSec(),
            cloud_queue_.front()->header.stamp.toSec());
        imu_queue_.pop();
        mutex_lock.unlock();
        return;
      }

      if (!cloud_queue_.empty() && cloud_queue_.front()->header.stamp.toSec()
          < imu_queue_.front()->header.stamp.toSec() - 0.5 * 0.1) {
        ROS_WARN(
            "odom_node: time stamp unaligned error and imu discarded, pls check your data; odom time %f, pc time %f",
            imu_queue_.front()->header.stamp.toSec(),
            cloud_queue_.front()->header.stamp.toSec());
        cloud_queue_.pop();
        mutex_lock.unlock();
        return;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in);
      ros::Time current_scan_time = (*cloud_queue_.front()).header.stamp;
      imu_queue_.pop();
      cloud_queue_.pop();
      mutex_lock.unlock();

      // 匹配
      OdomEstimate(pointcloud_in, current_scan_time);
    }
  } else {
    if (!cloud_queue_.empty()) {
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in);
      ros::Time current_scan_time = (*cloud_queue_.front()).header.stamp;
      cloud_queue_.pop();
      mutex_lock.unlock();
      // 匹配
      OdomEstimate(pointcloud_in, current_scan_time);
    }
  }
}
//}

void LidarOdom::OdomEstimate(const pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_scan_ptr,
                             const ros::Time &current_scan_time) {
  ros::Time test_time_1 = ros::Time::now();

  if (filtered_scan_ptr->empty()) {
    ROS_ERROR("check your cloud...");
    return;
  }
  transformed_scan_ptr->clear();

  ndt_start = ros::Time::now();
  if (initial_scan_loaded == 0) {
    // 点云转换到车体坐标系
    pcl::transformPointCloud(*filtered_scan_ptr, *transformed_scan_ptr, tf_l2b);  // tf_btol为初始变换矩阵
    localmap += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  ros::Time test_time_2 = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr
      localmap_ptr(new pcl::PointCloud<pcl::PointXYZI>(localmap));  // 用以保存降采样后的全局地图

  if (_method_type == MethodType::use_pcl) {
    pcl_ndt.setInputSource(filtered_scan_ptr);
    pcl_ndt.setInputTarget(localmap_ptr);
  } else if (_method_type == MethodType::use_cpu) {
    cpu_ndt.setInputSource(filtered_scan_ptr);
    cpu_ndt.setInputTarget(localmap_ptr);
  } else if (_method_type == MethodType::use_omp) {
    omp_ndt->setInputSource(filtered_scan_ptr);
    omp_ndt->setInputTarget(localmap_ptr);
  } else {
    ROS_ERROR("Please Define _method_type to conduct NDT");
    return;
  }

  ros::Time test_time_3 = ros::Time::now();  // TODO:

  // 计算初始姿态，上一帧点云结果+两针之间的匀速运动估计
//  guess_pose.x = previous_pose.x + diff_pose.x;  // 初始时diff_x等都为0
//  guess_pose.y = previous_pose.y + diff_pose.y;
//  guess_pose.z = previous_pose.z + diff_pose.z;
//  guess_pose.roll = previous_pose.roll;
//  guess_pose.pitch = previous_pose.pitch;
//  guess_pose.yaw = previous_pose.yaw + diff_pose.yaw;
  guess_pose = previous_pose + diff_pose;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.roll = previous_pose.roll;

  // 根据是否使用imu和odom,按照不同方式更新guess_pose(xyz,or/and rpy)
  // start2 只是为了把上面不同方式的guess_pose都标准化成guess_pose_for_ndt,为了后续操作方便
  pose guess_pose_for_ndt;
  if (_use_imu && _use_odom) {
    ImuOdomCalc(current_scan_time);
    guess_pose_for_ndt = guess_pose_imu_odom;
  } else if (_use_imu && !_use_odom) {
    ImuCalc(current_scan_time);
    guess_pose_for_ndt = guess_pose_imu;
  } else if (!_use_imu && _use_odom) {
    OdomCalc(current_scan_time);
    guess_pose_for_ndt = guess_pose_odom;
  } else {
    guess_pose_for_ndt = guess_pose;
  }
  // end2

  // start3 以下:根据guess_pose_for_ndt 来计算初始变换矩阵guess_init -- 针对TargetSource
  Eigen::AngleAxisf init_rotation_x(static_cast<const float &>(guess_pose_for_ndt.roll), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(static_cast<const float &>(guess_pose_for_ndt.pitch), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(static_cast<const float &>(guess_pose_for_ndt.yaw), Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(static_cast<const float &>(guess_pose_for_ndt.x),
                                        static_cast<const float &>(guess_pose_for_ndt.y),
                                        static_cast<const float &>(guess_pose_for_ndt.z));
  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_b2l;  // tf_btol
  // end3

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  // 用以保存ndt转换后的点云,align参数
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (_method_type == MethodType::use_pcl) {
    pcl_ndt.align(*output_cloud, init_guess);  // pcl::aligin 需传入转换后的点云(容器),估计变换
    fitness_score = pcl_ndt.getFitnessScore();
    t_localizer = pcl_ndt.getFinalTransformation();  // t_localizer为ndt变换得到的最终变换矩阵(即source和target之间的变换)
    has_converged = pcl_ndt.hasConverged();
    final_num_iteration = pcl_ndt.getFinalNumIteration();
    transformation_probability = pcl_ndt.getTransformationProbability();
  } else if (_method_type == MethodType::use_cpu) {
    cpu_ndt.align(*output_cloud, init_guess);            // cpu::align 只需要传入估计变换 --建图的时候传入估计变换,定位matching的时候传入空的单位Eigen
    fitness_score = cpu_ndt.getFitnessScore();
    t_localizer = cpu_ndt.getFinalTransformation();
    has_converged = cpu_ndt.hasConverged();
    final_num_iteration = cpu_ndt.getFinalNumIteration();
  } else if (_method_type == MethodType::use_omp) {
    omp_ndt->align(*output_cloud, init_guess);
    fitness_score = omp_ndt->getFitnessScore();
    t_localizer = omp_ndt->getFinalTransformation();
    has_converged = omp_ndt->hasConverged();
    final_num_iteration = omp_ndt->getFinalNumIteration();
  }
  ndt_end = ros::Time::now();
  ros::Time test_time_4 = ros::Time::now();  // TODO:

  // bask_link 需要排除掉全局起始点偏移造成的影响，全局起点偏移就是雷达起始有个高度和yaw偏角
  // t_localizer是相对位姿,t_base_link对应的是全局位姿
  t_base_link = t_localizer * tf_l2b;

  // 当前帧转换到全局地图上
  pcl::transformPointCloud(*filtered_scan_ptr, *transformed_scan_ptr, t_localizer);
  tf::Matrix3x3 mat_l, mat_b;  // 用以根据齐次坐标下的旋转变换,来求rpy转换角度
  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));


  // Update localizer_pose.  // 更新局部下的坐标
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update ndt_pose.  // 更新全局下的坐标
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  // current_pose 对应的是全局下的坐标!
  PointT this_pose_3d;
  PointTypePose this_pose_6d;
  current_pose = ndt_pose;
  this_pose_3d.x = ndt_pose.x;
  this_pose_3d.y = ndt_pose.y;
  this_pose_3d.z = ndt_pose.z;
  this_pose_3d.intensity = cloud_keyposes_3d_->points.size();  // 强度字段表示pose的index

  cloud_keyposes_3d_->points.push_back(this_pose_3d);

  this_pose_6d.x = this_pose_3d.x;
  this_pose_6d.y = this_pose_3d.y;
  this_pose_6d.z = this_pose_3d.z;
  this_pose_6d.intensity = this_pose_3d.intensity;
  this_pose_6d.roll = current_pose.roll;
  this_pose_6d.pitch = current_pose.pitch;
  this_pose_6d.yaw = current_pose.yaw;
  this_pose_6d.time = current_scan_time.toSec();
  cloud_keyposes_6d_->push_back(this_pose_6d);

  // 根据current和previous两帧之间的scantime,以及两帧之间的位置,计算两帧之间的变化
  // ros::Duration scan_duration = current_scan_time - previous_scan_time;
  double secs = (current_scan_time - previous_scan_time).toSec();
  // Calculate the offset (curren_pos - previous_pos)
  diff_pose = current_pose - previous_pose;
  diff = sqrt(diff_pose.x * diff_pose.x + diff_pose.y * diff_pose.y + diff_pose.z * diff_pose.z);

  current_velocity_x = diff_pose.x / secs;
  current_velocity_y = diff_pose.y / secs;
  current_velocity_z = diff_pose.z / secs;

  current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;

  current_velocity_imu_x = current_velocity_x;  // 修正imu速度
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // Update position and posture. current_pos -> previous_pos
  previous_pose = current_pose;
  previous_scan_time = current_scan_time;
  offset_imu_pose.init();
  offset_odom_pose.init();
  offset_imu_odom_pose.init();

  // Calculate the shift between added_pos and current_pos // 以确定是否更新全局地图
  // added_pose 将一直定位于localMap的原点
  ros::Time test_time_5 = ros::Time::now();
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift) {

    localmap_size += shift;
    odom_size += shift;

    // 只有在fitnessscore状态好的情况下才选取作为关键帧加入到localmap中
    // 这里的下采样网格大小将严重影响见图效果
    downSizeFilterLocalmap.setInputCloud(transformed_scan_ptr);
    downSizeFilterLocalmap.filter(*transformed_scan_ptr);

    localmap += *transformed_scan_ptr; // localmap内的距离达到阈值就清空,并重新从0开始一帧一帧添加点云
    submap += *transformed_scan_ptr;

    added_pose = current_pose;

    // 只是说map更新了,因此target也要更新,不要落后太多
    // 注意:此时加入的target:map_ptr并不包括刚加入点云的transformed_scan_ptr
    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(localmap_ptr);
    else if (_method_type == MethodType::use_cpu) {
      if (_incremental_voxel_update)
        cpu_ndt.updateVoxelGrid(transformed_scan_ptr);
      else
        cpu_ndt.setInputTarget(localmap_ptr);
    } else if (_method_type == MethodType::use_omp)
      omp_ndt->setInputTarget(localmap_ptr);
  }

  PublishCloud(current_scan_time);

  ros::Time test_time_6 = ros::Time::now();
  // end5

  // 当局部地图内的距离大于阈值,则清空localmap
  if (localmap_size >= max_localmap_size) {
    localmap = submap;
    submap.clear();
    localmap_size = 0.0;
  }

  std::cout << "*************************************************************************" << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of localmap points: " << localmap.size() << " points." << std::endl;
  std::cout << "Aligned Time: " << (ndt_end - ndt_start) * 1000 << " ms" << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "scan shift: " << shift << std::endl;
  std::cout << "localmap shift: " << localmap_size << std::endl;
  std::cout << "global path: " << odom_size << std::endl;
  //  std::cout << "1-2: " << (test_time_2 - test_time_1) << "ms" << "--downsample inputCloud" << std::endl;
  //  std::cout << "2-3: " << (test_time_3 - test_time_2) << "ms" << "--set params and inputSource" << std::endl;
  //  std::cout << "3-4: " << (test_time_4 - test_time_3) << "ms" << "--handle imu/odom and cal ndt resule"
  //            << std::endl;
  //  std::cout << "4-5: " << (test_time_5 - test_time_4) << "ms" << "--get current pose" << std::endl;
  //  std::cout << "5-6: " << (test_time_6 - test_time_5) << "ms" << "--publish current pose" << std::endl;
  std::cout << "*************************************************************************" << std::endl;
}

void LidarOdom::ImuCB(const sensor_msgs::ImuConstPtr &msg) {
//  std::lock_guard<std::mutex> lock(imu_data_mutex);
//  if (_imu_upside_down)  // _imu_upside_down指示是否进行imu的正负变换
//    imuUpSideDown(msg);

//  if (_use_imu) {
//    imu_data.push_back(input);
//  }

  mutex_lock.lock();
  imu_queue_.push(msg);
  mutex_lock.unlock();

  /*const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  // 解析imu消息,获得rpy
  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = warpToPmPi(imu_roll);  // 调整,防止超过PI(180°)  --保持在±180°内
  imu_pitch = warpToPmPi(imu_pitch);
  imu_yaw = warpToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);
  //

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x;
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

  imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;*/
}

void LidarOdom::OdomCB(const nav_msgs::OdometryConstPtr &msg) {
//  odom = *input;
//  OdomCalc(input->header.stamp);
  mutex_lock.lock();
  odom_queue_.push(msg);
  mutex_lock.unlock();
}

void LidarOdom::PcCB(const sensor_msgs::PointCloud2ConstPtr &msg) {
  mutex_lock.lock();
  cloud_queue_.push(msg);
  mutex_lock.unlock();
}

void LidarOdom::ImuOdomCalc(ros::Time current_time) {
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
  offset_imu_odom_pose.x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_pose.y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_pose.z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_pose.roll += diff_imu_roll;
  offset_imu_odom_pose.pitch += diff_imu_pitch;
  offset_imu_odom_pose.yaw += diff_imu_yaw;

  // ==> 最终的目的是融合imu和odom输出一个guess_pose
  // 注:guess_pose是在previous_pose基础上叠加一个offset,包括xyz的和rpy的
  // xyz的offset需要融合imu的转角和odom的速度(位移)
  // rpy的offset直接采用imu的rpy偏差值
  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_pose.x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_pose.y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_pose.z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_pose.roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pose.pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_pose.yaw;

  previous_time = current_time;
}

void LidarOdom::ImuCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  // 对imu由于不平衡造成的补偿问题,在这里解决
  // start1
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
  // end1

  // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
  offset_imu_pose.x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_pose.y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_pose.z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;  // imu的速度值会通过slam进行修正,以避免累计误差
  current_velocity_imu_y += accY * diff_time;  // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
  current_velocity_imu_z += accZ * diff_time;    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

  offset_imu_pose.roll += diff_imu_roll;
  offset_imu_pose.pitch += diff_imu_pitch;
  offset_imu_pose.yaw += diff_imu_yaw;

  guess_pose_imu.x = previous_pose.x + offset_imu_pose.x;
  guess_pose_imu.y = previous_pose.y + offset_imu_pose.y;
  guess_pose_imu.z = previous_pose.z + offset_imu_pose.z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_pose.roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pose.pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_pose.yaw;

  previous_time = current_time;
}

void LidarOdom::OdomCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_pose.x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_pose.y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_pose.z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_pose.roll += diff_odom_roll;
  offset_odom_pose.pitch += diff_odom_pitch;
  offset_odom_pose.yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose.x + offset_odom_pose.x;
  guess_pose_odom.y = previous_pose.y + offset_odom_pose.y;
  guess_pose_odom.z = previous_pose.z + offset_odom_pose.z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_pose.roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pose.pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_pose.yaw;

  previous_time = current_time;
}

void LidarOdom::imuUpSideDown(const sensor_msgs::Imu::Ptr input) {
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

//  input_yaw += M_PI/2;
  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

void LidarOdom::imu_info(const sensor_msgs::Imu &input) {
  const ros::Time current_time = input.header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  // 解析imu消息,获得rpy
  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input.orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = warpToPmPi(imu_roll);  // 调整,防止超过PI(180°)  --保持在±180°内
  imu_pitch = warpToPmPi(imu_pitch);
  imu_yaw = warpToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  imu.header = input.header;
  imu.linear_acceleration.x = input.linear_acceleration.x;
  // imu.linear_acceleration.y = input.linear_acceleration.y;
  // imu.linear_acceleration.z = input.linear_acceleration.z;
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

  ImuCalc(input.header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}

void LidarOdom::odom_info(const nav_msgs::Odometry &input) {
  OdomCalc(input.header.stamp);
}

void LidarOdom::SaveMap() {
  // 关闭终端时保存地图
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  std::string pcd_filename = map_saved_dir + "finalCLoud_" + stamp + ".pcd";
  if (!globalmap.empty()) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(globalmap, *map_cloud);
    pcl::VoxelGrid<pcl::PointXYZI> map_grid_filter;
    map_grid_filter.setLeafSize(1.0, 1.0, 1.0);
    map_grid_filter.setInputCloud(map_cloud);
    map_grid_filter.filter(*map_cloud);

    if (pcl::io::savePCDFileASCII(pcd_filename, *map_cloud) == -1) {
      std::cout << "Failed saving " << pcd_filename << "." << std::endl;
    }
    std::cout << "Saved globalmap " << pcd_filename << " (" << map_cloud->size() << " points)" << std::endl;
  }

  // 优化后的位姿也保存一份
  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(map_saved_dir + "trajectory_" + stamp + ".pcd", *poses);

}

void LidarOdom::ViewerThread() {
  ros::Rate rate(5);
  while (ros::ok()) {
    rate.sleep();
    // PublishCloud();
  }
}

void LidarOdom::PublishCloud(const ros::Time &current_scan_time) {
  if (transformed_scan_ptr->empty() || localmap.empty() || cloud_keyposes_3d_->empty()) {
    return;
  }

  // 发布关键帧位姿
/*  if (keyposes_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    keyposes_pub.publish(msg);
  }*/

  // 实时的点云也发布
  if (current_points_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr pointcloud_current_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*transformed_scan_ptr, *pointcloud_current_ptr);
    pointcloud_current_ptr->header.frame_id = "map";
    pointcloud_current_ptr->header.stamp = current_scan_time;
    current_points_pub.publish(*pointcloud_current_ptr);
  }

  /* pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(localmap, *local_cloud);
  downSizeFilterLocalmap.setInputCloud(local_cloud);
  downSizeFilterLocalmap.filter(*local_cloud);

  if (local_map_pub.getNumSubscribers() > 0) {
     sensor_msgs::PointCloud2::Ptr localmap_msg_ptr(new sensor_msgs::PointCloud2);
     pcl::toROSMsg(*local_cloud, *localmap_msg_ptr);
     localmap_msg_ptr->header.frame_id = "map";
     localmap_msg_ptr->header.stamp = current_scan_time;
     local_map_pub.publish(*localmap_msg_ptr);
   }
   local_cloud->clear();*/

  if (current_odom_pub.getNumSubscribers()) {

    Eigen::Quaternionf tmp_q(t_localizer.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

    static tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform tf_m2b;
    tf_m2b.setOrigin(
        tf::Vector3(t_localizer(0, 3), t_localizer(1, 3), t_localizer(2, 3)));
    tf_m2b.setRotation(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w()));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2b, current_scan_time, "map", "base_link"));

/*    Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d odom_in2 = Eigen::Isometry3d::Identity();
    odom_in.rotate(tmp_q.cast<double>());
    odom_in.pretranslate(Eigen::Vector3d(t_localizer(0, 3), t_localizer(1, 3), t_localizer(2, 3)));

    odom_in2.translate(Eigen::Vector3d(t_localizer(0, 3), t_localizer(1, 3), t_localizer(2, 3)));
    odom_in2.prerotate(tmp_q.cast<double>());

    std::cout << "t_localizaer: " << t_localizer << std::endl;
    std::cout << "t_base_link: " << t_base_link << std::endl;
    std::cout << "odom_in: " << odom_in.matrix() << std::endl;
    std::cout << "odom_in2: " << odom_in2.matrix() << std::endl;*/


    nav_msgs::Odometry odom;
    odom.header.stamp = current_scan_time;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = t_localizer(0, 3);
    odom.pose.pose.position.y = t_localizer(1, 3);
    odom.pose.pose.position.z = t_localizer(2, 3);

    odom.pose.pose.orientation.x = tmp_q.x();
    odom.pose.pose.orientation.y = tmp_q.y();
    odom.pose.pose.orientation.z = tmp_q.z();
    odom.pose.pose.orientation.w = tmp_q.w();

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = current_velocity_x;
    odom.twist.twist.linear.y = current_velocity_y;
    odom.twist.twist.angular.z = current_velocity_z;
    current_odom_pub.publish(odom);

    // 以下:对current_pose做tf变换,
/*    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    curr_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    transform.setRotation(curr_q);
    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "velo_link"));

    nav_msgs::Odometry odom;
    odom.header.stamp = current_scan_time;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = current_pose.x;
    odom.pose.pose.position.y = current_pose.y;
    odom.pose.pose.position.z = current_pose.z;

    odom.pose.pose.orientation.x = curr_q.x();
    odom.pose.pose.orientation.y = curr_q.y();
    odom.pose.pose.orientation.z = curr_q.z();
    odom.pose.pose.orientation.w = curr_q.w();

    odom.child_frame_id = "velo_link";
    odom.twist.twist.linear.x = current_velocity_x;
    odom.twist.twist.linear.y = current_velocity_y;
    odom.twist.twist.angular.z = current_velocity_z;
    current_odom_pub.publish(odom);*/
  }
}


