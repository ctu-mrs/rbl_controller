#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrackerCommand.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <mrs_msgs/Vec4.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h> #include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/Float64Stamped.h>
#include "rbl_controller/ActivateParams.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <filesystem>
#include <omp.h>
#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <deque>
#include <utility>
#include <chrono>
#include <mrs_octomap_planner/Path.h>
#include<optional>

class WrapperRosRBL : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool                                                is_initialized_ = false;
  bool               control_activated_ = false;
  bool               going_home_ = false;
  Eigen::Vector3d              angles_rpy_;
  double                         map_res_;

  std::mutex                             mtx_positions_;
  Eigen::Vector3d                        current_position_;
  std::vector<Eigen::Vector3d>                        group_positions_;

  std::mutex                             mtx_pcl_;
  pcl::PointCloud<pcl::PointXYZ>  cloud_;
  pcl::PointCloud<pcl::PointXYZ>  processed_cloud_;

  bool                                   _only_2d_ = false;
  double                                 _z_ref_;
  double                                 _z_min_;
  double                                 _z_max_;
  int                      _num_of_agents_;
  std::string              _agent_name_;
  std::string              _frame_;
  double                       _timeout_odom_;
  bool                         _use_livox_;
  double                       _livox_tilt_deg_;
  double                       _livox_fov_;
  Eigen::Vector3d              _livox_translation_;

  std::mutex                             mtx_rbl_;  
  std::vector<geometry_msgs::Point>      path_;
  Eigen::Vector3d                        target_{0, 0, 0};
  Eigen::Vector3d group_goal_{0, 0, 0};
  Eigen::Vector3d centroid_;
  double                                 radius_sense_;
    // RBL params
  double                                 _step_size_;
  double                                 _radius_;
  double                                 _encumbrance_;
  double                                 _dt_;
  double                                 __beta_min_;
  double                                 _betaD_;
  double                                 _beta_;
  double                                 _d1_;
  double                                 _d2_;
  double                                 _d3_;
  double                                 _d4_;
  double                                 _d5_;
  double                                 _d6_;
  double                                 _d7_;
  double                                 _th_;
  double                                 _ph_;
  double                                 _max_connection_dist_;
  double                                 _cwvd_rob_;
  double                                 _cwvd_obs_;
  double                                 _radius_search_;
  bool                                   _use_z_rule_;

  ros::ServiceServer srv_activate_control_;
  bool               cbSrvActivateControl([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  // ros::ServiceServer service_activate_params_control_;
  // bool               activationParamsServiceCallback(rbl_controller::ActivateParams::Request &req, rbl_controller::ActivateParams::Response &res);
  ros::ServiceServer srv_deactivate_control_;
  bool               cbSrvDeactivateControl([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  // ros::ServiceServer service_save_to_csv_;
  ros::ServiceServer srv_goto_position_;
  bool cbSrvGotoPosition(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
  ros::ServiceServer srv_go_home_;
  bool               cbSrvGoHome([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  std::mutex mutex_srv_;
  ros::ServiceClient       sc_get_path_;
  ros::ServiceClient       sc_set_ref_;
  ros::ServiceClient sc_goto_position_;

  ros::Timer               tm_set_ref_;
  void                     cbTmSetRef([[maybe_unused]] const ros::TimerEvent &te);
  ros::Timer               tm_update_target_;
  void cbTmUpdateTarget([[maybe_unused]] const ros::TimerEvent&);

  ros::Timer tm_diagnostics_;
  void       cbTmDiagnostics([[maybe_unused]] const ros::TimerEvent &te);

  ros::Publisher                 pub_viz_pcl_;
std::shared_ptr<sensor_msgs::PointCloud2> getVizPcl(const pcl::PointCloud<pcl::PointXYZ>& pcl, const std::string& frame);
  ros::Publisher                 pub_viz_cell_A_;
  ros::Publisher                 pub_viz_cell_A_sensed_;
std::shared_ptr<sensor_msgs::PointCloud2> getVizCellA(const std::vector<Eigen::Vector3d>& points, const std::string& frame);
  ros::Publisher                 pub_viz_planes_;
visualization_msgs::MarkerArray getVizPlanes(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& planes, const std::string& frame);
  ros::Publisher                 pub_viz_path_;
nav_msgs::Path getVizPath(const std::vector<Eigen::Vector3d>& path, const std::string& frame);
  ros::Publisher pub_viz_position_;
  visualization_msgs::Marker            getVizPosition(const Eigen::Vector3d& point, const double scale, const std::string& frame);
  ros::Publisher pub_viz_centroid_;
  visualization_msgs::Marker            getVizCentroid(const Eigen::Vector3d& point, const std::string& frame);
  ros::Publisher pub_viz_target_;
  visualization_msgs::Marker            getVizModGroupGoal(const Eigen::Vector3d& point, const double scale, const std::string& frame);

  mrs_lib::SubscribeHandler sh_traj_;
  void cbSubTraj(const visualization_msgs::MarkerArray::ConstPtr& msg);

  mrs_lib::SubscribeHandler sh_uvdar_poses_;
  void                         cbSubUvdar(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &msg);

  std::vector<mrs_lib::SubscribeHandler> sh_group_odoms_;
  void                         cbSubGroupOdom(const nav_msgs::OdometryConstPtr &msg, int idx); 

  mrs_lib::SubscribeHandler                        sh_odom_;
  void                         cbSubOdom(const nav_msgs::Odometry::ConstPtr &msg);

  mrs_lib::SubscribeHandler                 sh_pcl_;
  void cbSubPCL(const sensor_msgs::PointCloud2& pcl_cloud2);

  std::vector<Eigen::Vector3d> pointCloudToEigenVector(const pcl::PointCloud<pcl::PointXYZ>& cloud); 

  std::shared_ptr<mrs_lib::Transformer> transformer_;

std::optional<std::vector<geometry_msgs::Point>> getPath(const Eigen::Vector3d& start,
                                                         const Eigen::Vector3d& end);
};

void WrapperRosRBL::onInit() {
  ros::NodeHandle &nh = getPrivateNodeHandle();
  NODELET_DEBUG("Initializing nodelet...");
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "WrapperRosRBL");

  param_loader.loadParam("uav_name", _agent_name_);
  param_loader.loadParam("only_2d", _only_2d_);
  param_loader.loadParam("control_frame", _frame_);
  param_loader.loadParam("num_of_agents", _num_of_agents_);
  param_loader.loadParam("timeout", _timeout_odom_);
  param_loader.loadParam("z_min", _z_min_);
  param_loader.loadParam("z_max", _z_max_);
  param_loader.loadParam("z_ref", _z_ref_);
  param_loader.loadParam("livox/enabled", _use_livox_);
  param_loader.loadParam("livox/tilt_deg", _livox_tilt_deg_);
  param_loader.loadParam("livox/fov", _livox_fov_);
  auto livox_wrt_uav = param_loader.loadParam2("livox/translation");
  auto odom_topic_name = param_loader.loadParam2("odometry_topic");
  auto rate_tm_set_ref = param_loader.loadParam2("rate/timer_set_ref");
  auto rate_tm_diagnostics = param_loader.loadParam2("rate/timer_diagnostics");

  param_loader.loadParam("rbl_controller/d1", _d1_);
  param_loader.loadParam("rbl_controller/d2", _d2_);
  param_loader.loadParam("rbl_controller/d3", _d3_);
  param_loader.loadParam("rbl_controller/d4", _d4_);
  param_loader.loadParam("rbl_controller/d5", _d5_);
  param_loader.loadParam("rbl_controller/d6", _d6_);
  param_loader.loadParam("rbl_controller/d7", _d7_);
  param_loader.loadParam("rbl_controller/radius", _radiu_s);
  param_loader.loadParam("rbl_controller/encumbrance", encumbrance);
  param_loader.loadParam("rbl_controller/step_size", _step_size_);
  param_loader.loadParam("rbl_controller/betaD", betaD);
  param_loader.loadParam("rbl_controller/beta", _beta_);
  param_loader.loadParam("rbl_controller/_beta_min", beta_min);
  param_loader.loadParam("rbl_controller/use_z_rule", _use_z_rule_);
  param_loader.loadParam("rbl_controller/dt", dt);
  param_loader.loadParam("rbl_controller/max_connection_dist", _max_connection_dist_);
  param_loader.loadParam("rbl_controller/cwvd_rob", _cwvd_rob_);
  param_loader.loadParam("rbl_controller/cwvd_obs", _cwvd_obs_);
  param_loader.loadParam("rbl_controller/radius_search", _radius_search_); 

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WrapperRosRBL]: Could not load all parameters!");
    ros::shutdown();
  }

  _livox_translation_ = Eigen::Vector3d{livox_wrt_uav[0], livox_wrt_uav[1], livox_wrt_uav[2]};
  radius_sense_ = _radius_ / _cwvd_obs_ + _radius_search_;

  auto it = std::find(_uav_names_.begin(), _uav_names_.end(), _agent_name_);

  if (it != _uav_names_.end()) {
    this_uav_idx_ = it - _uav_names_.begin();
    _uav_names_.erase(it);
    _uvdar_ids_.erase(_uvdar_ids_.begin() + this_uav_idx_);
  } else {
    ROS_ERROR("[WrapperRosRBL]: This UAV is not part of the formation! Check the config file. Shutting down node.");
    ros::shutdown();
  }
  beta      = betaD;
  _num_of_agents_ = _uav_names_.size();

  group_positions_.resize(_num_of_agents_);
  uav_positions_.resize(_num_of_agents_);
  neighbors_and_obstacles_noisy.resize(_num_of_agents_);

  goal[0]          = target[0];
  goal[1]          = target[1];
  goal[2]          = target[2];
  goal_original[0] = target[0];
  goal_original[1] = target[1];
  goal_original[2] = target[2];

  // std::vector<Eigen::Vector3d> uav_positionsN_(uav_positions_);
  /* create multiple subscribers to read uav odometries */
  // iterate through drones except this drone and target
  for (int i = 0; i < _uav_names_.size(); i++) {
    std::string topic_name = std::string("/") + _uav_names_[i] + std::string("/") + _odometry_topic_name_; 
    sh_group_odoms_.push_back(nh.subscribe<nav_msgs::Odometry>(topic_name.c_str(), 1, boost::bind(&WrapperRosRBL::cbSubGroupOdom, this, _1, i)));
    _uav_uvdar_ids_[_uvdar_ids_[i]] = i;
  } 
  sh_odom_ = nh.subscribe("/" + _agent_name_ + "/estimation_manager/odom_main", 1, &WrapperRosRBL::cbSubOdom, this);
  sh_uvdar_poses_.push_back(nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/" + _agent_name_ + "/uvdar/measuredPoses", 1, boost::bind(&WrapperRosRBL::cbSubUvdar, this, _1)));
  mrs_lib::SubscribeHandlerOptions shopts;
  //
  shopts.nh                 = nh;
  shopts.node_name          = "WrapperRosRBL";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_position_command_ = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");
    sh_traj_ = nh.subscribe<visualization_msgs::MarkerArray>("/" + _agent_name_ + "/trajectory_generation/markers/final", 1, &WrapperRosRBL::cbSubTraj, this);
    
  // initialize timers
  tm_set_ref_ = nh.createTimer(ros::Rate(rate_tm_set_ref), &WrapperRosRBL::cbTmSetRef, this);
  tm_update_target_ = nh.createTimer(ros::Rate(rate_tm_set_ref), &WrapperRosRBL::cbTmUpdateTarget, this);
  tm_diagnostics_   = nh.createTimer(ros::Rate(_rate_timer_diagnostics_), &WrapperRosRBL::cbTmDiagnostics, this);
  // timer_pub_ = nh.createTimer(ros::Rate(rate_tm_set_ref), &WrapperRosRBL::callbackPublisher, this);

  // initialize service servers
  srv_activate_control_   = nh.advertiseService("control_activation_in", &WrapperRosRBL::cbSrvActivateControl, this);
  srv_deactivate_control_ = nh.advertiseService("control_deactivation_in", &WrapperRosRBL::cbSrvDeactivateControl, this);
  // service_activate_params_control_   = nh.advertiseService("control_activation_params_in", &WrapperRosRBL::activationParamsServiceCallback, this);
  srv_go_home_       = nh.advertiseService("fly_to_start_in", &WrapperRosRBL::cbSrvGoHome, this);
  service_save_to_csv_        = nh.advertiseService("save_to_csv_in", &WrapperRosRBL::saveToCsvServiceCallback, this);
  srv_goto_position_ = nh.advertiseService("goto_in", &WrapperRosRBL::cbSrvGotoPosition, this);

  // initialize service clients
  sc_set_ref_  = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("ref_pos_out");
  sc_goto_position_ = nh.serviceClient<mrs_msgs::Vec4>("goto_out");
  sc_get_path_ = nh.serviceClient<mrs_octomap_planner::Path>("get_path_out");
  // sc_traj_gen_ = nh.serviceClient<mrs_octomap_planner::Path>("get_path_out");

  // initialize publishers
  pub_viz_position_       = nh.advertise<visualization_msgs::Marker>("position_out", 1, true);
  pub_viz_centroid_       = nh.advertise<visualization_msgs::Marker>("centroid_out", 1, true);
  pub_viz_pcl_     = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);
  pub_viz_cell_A_          = nh.advertise<sensor_msgs::PointCloud2>("cell_A", 1, true);
  pub_viz_cell_A_sensed_  = nh.advertise<sensor_msgs::PointCloud2>("actively_sensed_A", 1, true);
  pub_viz_norms_          = nh.advertise<visualization_msgs::MarkerArray>("planes_norms", 1, true);
  pub_viz_path_           = nh.advertise<nav_msgs::Path>("path", 1, true);
  pub_viz_target_ = nh.advertise<visualization_msgs::Marker>("viz/target", 1, true);

  if (simulation_) {
    // sh_pcl_  = nh.subscribe("/" + _agent_name_ + "/livox_fake_scan", 1, &WrapperRosRBL::cbSubPCL, this);
    sh_pcl_  = nh.subscribe("/" + _agent_name_ + "/octomap_local_vis/octomap_point_cloud_centers", 1, &WrapperRosRBL::cbSubPCL, this);
  } else { 
    if (use_bonxai_mapping) {
      // sh_pcl_  = nh.subscribe("/" + _agent_name_ + "/bonxai_server/local_pc", 1, &WrapperRosRBL::cbSubPCL, this);
      sh_pcl_  = nh.subscribe("/" + _agent_name_ + "/octomap_local_vis/octomap_point_cloud_centers", 1, &WrapperRosRBL::cbSubPCL, this);
    } else {
      sh_pcl_  = nh.subscribe("/" + _agent_name_ + "/pcl_filter/livox_points_processed", 1, &WrapperRosRBL::cbSubPCL, this);
    }
  }
  sub_neighbors_ = nh.subscribe("/" + _agent_name_ + "/filter_reflective_uavs/agents_pcl", 1, &WrapperRosRBL::neighborCallback, this);

  // initialize transformer
  transformer_ = std::make_shared<mrs_lib::Transformer>(nh, "WrapperRosRBL");
  transformer_->retryLookupNewest(true);

  is_initialized_ = true;
  ROS_INFO("[WrapperRosRBL]: Initialization completed.");
}

std::vector<Eigen::Vector3d> WrapperRosRBL::pointCloudToEigenVector(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.reserve(cloud.size());

  for (const auto& pt : cloud.points) {
    eigen_points.emplace_back(pt.x, pt.y, pt.z);
  }

  return eigen_points;
}

void WrapperRosRBL::cbTmUpdateTarget(const ros::TimerEvent&)
{  
  if (!is_initialized_) {
    return;
  }
  

    ROS_WARN_THROTTLE(3.0, "[WrapperRosRBL]: Waiting for activation.");
    return;
  }

  auto points_copy  = mrs_lib::get_mutexed(mtx_path_, path_);
  auto uav_position = mrs_lib::get_mutexed(mutex_position_command_, uav_position_);
  auto group_goal   = mrs_lib::get_mutexed(mutex_position_command_, group_goal_);
  auto centroid     = mrs_lib::get_mutexed(mutex_centroid_, centroid_);

  if (points_copy.size() < 2) {
    ROS_WARN_STREAM("[WrapperRosRBL]: Can not select a new goal, path length = " << points_copy.size());
    return;
  }

  if (isReplanNeeded(uav_position, points_copy, centroid)) {
    ROS_INFO_STREAM("[WrapperRosRBL]: Replanning");

    auto ret = getPath(uav_position, group_goal);
    {
      std::scoped_lock lck(mtx_path_);
      if (ret) {
        path_ = getInterpolatedPath(ret.value(), 0.2);
        points_copy   = path_;

        pub_viz_path_.publish(getVizPath(path_));
      }
    }
  }

  // Step 1: Find the closest point on the path to the current robot position
  size_t start_idx = 0;
  double min_dist  = std::numeric_limits<double>::max();
  for (size_t i = 0; i < points_copy.size(); ++i) {
    double dx   = points_copy[i].x - current_position[0];
    double dy   = points_copy[i].y - current_position[1];
    double dz   = points_copy[i].z - current_position[2];
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < min_dist) {
      min_dist  = dist;
      start_idx = i;
    }
  }

  // Step 2: Walk forward along the path and accumulate distance
  const double target_distance      = radius;
  double       accumulated_distance = 0.0;
  size_t       best_idx             = start_idx;

  for (size_t i = start_idx; i < points_copy.size() - 1; ++i) {
    const geometry_msgs::Point& p1      = points_copy[i];
    const geometry_msgs::Point& p2      = points_copy[i + 1];
    double                      dx      = p2.x - p1.x;
    double                      dy      = p2.y - p1.y;
    double                      dz      = p2.z - p1.z;
    double                      segment = std::sqrt(dx * dx + dy * dy + dz * dz);
    accumulated_distance += segment;

    if (accumulated_distance >= target_distance) {
      best_idx = i + 1;
      break;
    }
  }

  const geometry_msgs::Point& chosen = points_copy[best_idx];
  target[0] = goal[0] = chosen.x;
  target[1] = goal[1] = chosen.y;
  target[2] = goal[2] = chosen.z;

  ROS_DEBUG("Goal updated at path distance ~4m: idx=%lu, x=%.2f y=%.2f z=%.2f", best_idx, chosen.x, chosen.y, chosen.z);
}

void WrapperRosRBL::cbSubGroupOdom(const nav_msgs::OdometryConstPtr &msg, int idx) {

  // if (!is_initialized_)
  // {
  //   return;
  // }

  /* nav_msgs::Odometry msg = *sh_ptr.getMsg(); */

  if ((ros::Time::now() - msg->header.stamp).toSec() > _timeout_odom_)
  {
    ROS_WARN("[WrapperRosRBL]: The latency of odom message for %s exceeds the threshold (latency = %.2f s).", _uav_names_[idx].c_str(),
             (ros::Time::now() - msg->header.stamp).toSec());
  }

  geometry_msgs::PointStamped new_point;

  new_point.header = msg->header;
  new_point.point.x = msg->pose.pose.position.x;
  new_point.point.y = msg->pose.pose.position.y;
  new_point.point.z = msg->pose.pose.position.z;
  // std::cout <<"x: "<< new_point.point.x  <<"y: "<<new_point.point.y<< std::endl;
  // auto res = transformer_->transformSingle(new_point, _frame_);
  // if (res)
  // {
  //   new_point = res.value();
  // }
  // else
  // {
  //   ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform odometry msg to control frame.");
  //   return;
  // }

  // std::cout <<"x: "<< new_point.point.x <<"y:" <<new_point.point.y<< std::endl;
  Eigen::Vector3d transformed_position;
  if (_c_dimensions_ == 3)
  {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
  }
  else
  {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
  }

  mrs_lib::set_mutexed(mtx_positions_, transformed_position, uav_positions_[idx]);
  mrs_lib::set_mutexed(mtx_positions_, transformed_position, group_positions_[idx]);

}

void WrapperRosRBL::cbSubOdom(const nav_msgs::Odometry::ConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  geometry_msgs::PointStamped new_point;


  new_point.header  = msg->header;
  new_point.point.x = msg->pose.pose.position.x;
  new_point.point.y = msg->pose.pose.position.y;
  new_point.point.z = msg->pose.pose.position.z;

  double q_x = msg->pose.pose.orientation.x;
  double q_y = msg->pose.pose.orientation.y;
  double q_z = msg->pose.pose.orientation.z;
  double q_w = msg->pose.pose.orientation.w;
  
  Eigen::Vector3d transformed_position;
  double angle_rad = 0.0;
  if (_c_dimensions_ == 3) {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
  } else {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
  }

  Eigen::Vector3d euler; // roll, pitch, yaw

  // Roll (X-axis rotation)
  double sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
  double cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
  euler.x() = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (Y-axis rotation)
  double sinp = 2.0 * (q_w * q_y - q_z * q_x);
  if (std::abs(sinp) >= 1){
    euler.y() = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
  } else {
    euler.y() = std::asin(sinp);
  }

  // Yaw (Z-axis rotation)
  double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
  double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
  euler.z() = std::atan2(siny_cosp, cosy_cosp);

  mrs_lib::set_mutexed(mtx_positions_, transformed_position, uav_position_);
  mrs_lib::set_mutexed(mtx_positions_, euler, angles_rpy_);

}

void WrapperRosRBL::cbSubTraj(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    if (!msg->markers.empty() && msg->markers[0].points.size() > 1) {

        std::lock_guard<std::mutex> lock(mtx_path_);
        const visualization_msgs::Marker& marker0 = msg->markers[0];
        const std::vector<geometry_msgs::Point>& input_points = marker0.points;
        std::vector<geometry_msgs::Point> dense_points;

        const double interpolation_resolution = 0.2;  // meters between interpolated points

        // Interpolate between each consecutive pair
        for (size_t i = 0; i < input_points.size() - 1; ++i) {
            const geometry_msgs::Point& p1 = input_points[i];
            const geometry_msgs::Point& p2 = input_points[i + 1];

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double dz = p2.z - p1.z;

            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            int steps = std::max(1, static_cast<int>(dist / interpolation_resolution));

            for (int j = 0; j <= steps; ++j) {
                double t = static_cast<double>(j) / steps;
                geometry_msgs::Point interp;
                interp.x = p1.x + t * dx;
                interp.y = p1.y + t * dy;
                interp.z = p1.z + t * dz;
                dense_points.push_back(interp);
            }
        }

        // ROS_INFO("Interpolated to %lu total points", dense_points.size());
        // const geometry_msgs::Point& chosen = dense_points[best_idx];
        path_ = dense_points;

    } else {
        ROS_WARN("Not enough points to interpolate.");
    }
}

/*WrapperRosRBL::cbSubUvdar() //{ */
void WrapperRosRBL::cbSubUvdar(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &array_poses) {

  // ROS_INFO("Received pose from uvdar");
  /* group_positions_.assign(_num_of_agents_ + 1, Eigen::Vector3d(1000, 0, 0)); */
  Eigen::Matrix2d CovarianceMatrix;
  for (int i = 0; i < array_poses->poses.size(); i++) {
    /* create new msg */
    geometry_msgs::PointStamped new_point;

    Eigen::Vector3d transformed_position;
    new_point.header  = array_poses->header;
    new_point.point.x = array_poses->poses[i].pose.position.x;
    new_point.point.y = array_poses->poses[i].pose.position.y;
    new_point.point.z = array_poses->poses[i].pose.position.z;
    int uav_id        = array_poses->poses[i].id;
    CovarianceMatrix << array_poses->poses[i].covariance[0], array_poses->poses[i].covariance[1], array_poses->poses[i].covariance[6],
        array_poses->poses[i].covariance[7];

    auto res = transformer_->transformSingle(new_point, _frame_);
    if (res) {
      new_point = res.value();
    } else {
      ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: UVDAR: Could not transform positions to control frame.");
      continue;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(CovarianceMatrix);
    Eigen::Vector2d                                eigenvalues        = solver.eigenvalues();
    double                                         largest_eigenvalue = eigenvalues.maxCoeff();
    double valeig  = std::abs(std::abs(std::sqrt(pow((new_point.point.x - position_command_.x), 2) + pow((new_point.point.y - position_command_.y), 2))) / 2);
    double valeig1 = 2.6 * std::sqrt(largest_eigenvalue) + encumbrance;

    /* std::cout << "Largest eigenvalue: " << uav_id << " eig: " << largest_eigenvalue << std::endl; */

    if (_c_dimensions_ == 3) {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    } else {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
    }
    if (largest_eigenvalue < 15.0) {
      mrs_lib::set_mutexed(mutex_uav_uvdar_, transformed_position, group_positions_[_uav_uvdar_ids_[uav_id]]);
    }
  }
}

void WrapperRosRBL::cbTmSetRef([[maybe_unused]] const ros::TimerEvent &te) {
  if (!is_initialized_) {
    return;
  }




    ROS_WARN_THROTTLE(3.0, "[WrapperRosRBL]: Waiting for activation.");
    return;
  }

  mrs_msgs::Reference p_ref;

  {
    std::scoped_lock                       lock(mtx_positions_, mutex_position_command_, mutex_uav_uvdar_);
    auto                                   start = std::chrono::steady_clock::now();
    // std::vector<Eigen::Vector3d> neighbors;
    std::vector<Eigen::Vector3d> neighbors_and_obstacles;
    if (_only_2d_){
      current_position = {uav_position_[0], uav_position_[1], uav_position_[2]};
    } else {
      current_position = {uav_position_[0], uav_position_[1], _z_ref_};
    }

    for (int j = 0; j < _num_of_agents_; ++j) {
      if (_only_2d_) {
        // neighbors.push_back(Eigen::Vector3d{group_positions_[j][0], group_positions_[j][1], group_positions_[j][2]});
        neighbors_and_obstacles.push_back(Eigen::Vector3d{group_positions_[j][0], group_positions_[j][1], group_positions_[j][2]});
      } else {
        // neighbors.push_back(Eigen::Vector3d{group_positions_[j][0], group_positions_[j][1], _z_ref_});
        neighbors_and_obstacles.push_back(Eigen::Vector3d{group_positions_[j][0], group_positions_[j][1], _z_ref_});
      }
    }

    { 
      size_neighbors_and_obstacles.clear();
      size_obstacles.assign(obstacles_.size(), size_obstacles1);
    }
    size_neighbors_and_obstacles = size_neighbors;  // Copy vec1 to vec3
    size_neighbors_and_obstacles.insert(size_neighbors_and_obstacles.end(), size_obstacles.begin(), size_obstacles.end());

    target_ = target;

    auto msg_target_ = geometry_msgs::PointStamped();
    msg_target_.header.stamp = ros::Time::now();
    msg_target_.header.frame_id = _frame_;

    msg_target_.point.x = target_(0);
    msg_target_.point.y = target_(1);
    msg_target_.point.z = target_(2);
    pub_viz_target_.publish(msg_target_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.makeShared();
    if (cloud_ptr && !cloud_ptr->empty()) { 
      if (save_scene_to_csv) {
        std::cout << "saving raw point cloud to csv" << std::endl;
        savePCL2TOCSV(cloud_ptr, "raw_pointcloud.csv");
      }

      if (use_bonxai_mapping) {
        processed_cloud = *cloud_ptr;
      } else {
        processed_cloud = voxelize_pcl(cloud_ptr, map_res_);
      }

    }

    std::vector<Eigen::Vector3d> path_points;
    path_points.push_back(current_position);
    path_points.push_back(target);

    // getVizPath(path_points);

    auto centroids = WrapperRosRBL::get_centroid(current_position, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles, size_neighbors_and_obstacles,
                                        encumbrance, target_, beta, neighbors_and_obstacles_noisy);
    auto centroids_no_rotation = WrapperRosRBL::get_centroid(current_position, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles, size_neighbors_and_obstacles, encumbrance,
                                    {goal[0], goal[1], goal[2]}, beta, neighbors_and_obstacles_noisy);

    std::vector<double> c1         = {std::get<0>(centroids).x(), std::get<0>(centroids).y(), std::get<0>(centroids).z()};
    std::vector<double> c2         = {std::get<1>(centroids).x(), std::get<1>(centroids).y(), std::get<1>(centroids).z()};
    std::vector<double> c1_no_conn = {std::get<2>(centroids).x(), std::get<2>(centroids).y(), std::get<2>(centroids).z()};

    std::vector<double> c1_no_rotation = {std::get<2>(centroids_no_rotation).x(), std::get<2>(centroids_no_rotation).y(), std::get<2>(centroids_no_rotation).z()};

    std::vector<double> current_position(3);  // Vector with two elements
    current_position[0] = current_position[0];    // Assigning first element
    current_position[1] = current_position[1];
    current_position[2] = current_position[2];

    apply_rules(beta, c1_no_conn, c2, current_position, dt, _beta_min, betaD, goal, d1, th, d2, d3, d4, d5, d6, d7, target, c1_no_rotation);

    mrs_lib::set_mutexed(mutex_centroid_, Eigen::Vector3d{ c1_no_conn[0], c1_no_conn[1], c1_no_conn[2] }, centroid_);
    c1_to_rviz = c1_no_conn;

    if (_use_livox_) {
      double desired_heading = std::atan2(c1_no_conn[1] - current_position[1], c1_no_conn[0] - current_position[0]);
      p_ref.heading = desired_heading;
      double diff = std::fmod(desired_heading - angles_rpy_[2] + M_PI, 2 * M_PI) - M_PI;
      double difference = (diff < -M_PI) ? diff + 2 * M_PI : diff;

      if (std::abs(difference) < M_PI/2) {
        p_ref.position.x = c1_no_conn[0];  // next_values[0];
        p_ref.position.y = c1_no_conn[1];
        p_ref.position.z = c1_no_conn[2];
      } else {
        p_ref.position.x = current_position[0];  // next_values[0];
        p_ref.position.y = current_position[1];
        p_ref.position.z = current_position[2];
      }
    } else {
      p_ref.position.x = c1_no_conn[0];  // next_values[0];
      p_ref.position.y = c1_no_conn[1];
      p_ref.position.z = c1_no_conn[2];
    }

    saveUAVPositionToCSV(_agent_name_, current_position, "uav_positions.csv");

  }

  if (save_scene_to_csv) {
    std::cout << "All saved setting save_scene_to_csv as false" << std::endl;
    save_scene_to_csv = false;
  }

  // set drone ref
  mrs_msgs::ReferenceStampedSrv srv;
  srv.request.reference       = p_ref;
  srv.request.header.frame_id = _frame_;
  srv.request.header.stamp    = ros::Time::now();

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  if (sc_set_ref_.call(srv)) {

  } else {
    ROS_ERROR_THROTTLE(3.0, "Failed to call service ref_pos_out");
  }
  if (!flag_stop && current_position[1] > 0) {
    flag_stop                    = true;
    ros::Time     end_time_1     = ros::Time::now();
    ros::Duration elapsed_time_1 = end_time_1 - start_time_1;
    // ROS_INFO("Elapsed time: %.2f seconds", elapsed_time_1.toSec());
  }
}

void WrapperRosRBL::cbTmDiagnostics([[maybe_unused]] const ros::TimerEvent &te) {

  if (!is_initialized_) {
    return;
  }

  bool timeout_exceeded = false;

  try {
  auto mod_group_goal = mrs_lib::get_mutexed(mtx_positions_, group_goal_);
    pub_viz_target_.publish(getVizModGroupGoal(mod_group_goal, 0.5));
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_viz_target_.getTopic().c_str());
  }

  try {
  auto poistion = mrs_lib::get_mutexed(mtx_positions_,poistion_);
    pub_viz_position_.publish(getVizPosition(poistion, 0.5));
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_viz_position_.getTopic().c_str());
  }

  try {
    getVizPcl();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_viz_pcl_.getTopic().c_str());
  }

  try {
  auto centroid = mrs_lib::get_mutexed(mtx_positions_,centroid_);
    pub_viz_centroid_.publish(getVizCentroid(centroid, 0.5));
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_viz_centroid_.getTopic().c_str());
  }

}
//}

void WrapperRosRBL::cbSubPCL(const sensor_msgs::PointCloud2& pcl_cloud2) {
  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>); 

  pcl::fromROSMsg(pcl_cloud2, *temp_cloud);

  cloud = *temp_cloud;
  /* ROS_INFO_STREAM("Received point cloud with " << cloud.size() << "points."); */
}

bool WrapperRosRBL::cbSrvActivateControl(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for activation of planning
  
  res.success = true;


    res.message = "Control was already allowed.";
    ROS_WARN("[WrapperRosRBL]: %s", res.message.c_str());
  } else {
    
    res.message      = "Control allowed.";
    ROS_INFO("[WrapperRosRBL]: %s", res.message.c_str());
  }

  start_time_1 = ros::Time::now();
  ROS_INFO("Start time activation: %.2f seconds", start_time_1.toSec());
  return true;
}

bool WrapperRosRBL::cbSrvDeactivateControl(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  
  ROS_INFO("[WrapperRosRBL]: Deactivation service called.");
  res.success = true;


    res.message = "Control was already disabled.";
    ROS_WARN("[WrapperRosRBL]: %s", res.message.c_str());
  } else {
    
    res.message      = "Control disabled.";
    ROS_INFO("[WrapperRosRBL]: %s", res.message.c_str());
  }

  return true;
}

bool WrapperRosRBL::cbSrvGoHome(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for activation of planning
  ROS_INFO("[WrapperRosRBL]: Fly to start service called.");
  res.success = true;

  mrs_msgs::Vec4 srv;
  srv.request.goal = {_required_initial_position_[0], _required_initial_position_[1], _required_initial_position_[2], 0.0};
  ;
  if (sc_goto_position_.call(srv)) {
    if (srv.response.success) {
      res.message          = "Fly to start called.";
      going_home_ = true;
    } else {
      res.success = false;
      res.message = "GoTo service returned success false.";
    }
  } else {
    res.success = false;
    res.message = "Call to GoTo service failed.";
  }

  ROS_INFO("[WrapperRosRBL]: %s", res.message.c_str());


  return true;
}

bool WrapperRosRBL::cbSrvGotoPosition(mrs_msgs::Vec4::Request&  req,
                                        mrs_msgs::Vec4::Response& res)
{
    res.success = false;
    res.message = "Node not ready";
    return false;
  // }
      auto uav_position                = mrs_lib::get_mutexed(mutex_position_command_, uav_position_);

      if (!ret) {auto goto_goal = Eigen::Vector3d{req.goal[0], req.goal[1], req.goal[2]};
      
      control_activated_ = true;auto ret = getPath(uav_position, goto_goal);
  {
    std::scoped_lock lck(mtx_path_);
    
      path_.clear();
      res.success = false;
      res.message = "Path not found";
      return false;
    }
    path_ = getInterpolatedPath(ret.value(), 0.2);
    group_goal_ = goto_goal;

        pub_viz_path_.publish(getVizPath(path_));
  }
  
  res.success = true;
  res.message = "Path found";
  return true;
}

// | -------------------- nodelet macro ----------------------- |
PLUGINLIB_EXPORT_CLASS(formation_control::WrapperRosRBL, nodelet::Nodelet);
}  // namespace formation_control
