#include <rbl_controller.h>


namespace formation_control
{
// Get the start time
/*RBLController::onInit () //{ */
void RBLController::onInit() {
  // initialize nodelet
  ros::NodeHandle &nh = getPrivateNodeHandle();
  NODELET_DEBUG("Initializing nodelet...");
  ros::Time::waitForValid();

  // load parameters
  std::string          _leader_name;
  std::vector<int>     _uvdar_ids_;
  mrs_lib::ParamLoader param_loader(nh, "RBLController");

  /*RBLController::LoadPram //{ */
  param_loader.loadParam("uav_names", _uav_names_);
  param_loader.loadParam("uav_uvdar_ids", _uvdar_ids_);
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("odometry_topic", _odometry_topic_name_);
  param_loader.loadParam("set_reference_timer/rate", _rate_timer_set_reference_);
  param_loader.loadParam("control_frame", _control_frame_);
  param_loader.loadParam("odom_msg_max_latency", _odom_msg_max_latency_);
  param_loader.loadParam("diagnostics/odom_timeout", _odom_timeout_);
  param_loader.loadParam("diagnostics/rate", _rate_timer_diagnostics_);
  param_loader.loadParam("d1", d1);
  param_loader.loadParam("d2", d2);
  param_loader.loadParam("d3", d3);
  param_loader.loadParam("d4", d4);
  param_loader.loadParam("d5", d5);
  param_loader.loadParam("d6", d6);
  param_loader.loadParam("d7", d7);
  param_loader.loadParam("radius", radius);
  param_loader.loadParam("encumbrance", encumbrance);
  param_loader.loadParam("step_size", step_size);
  param_loader.loadParam("betaD", betaD);
  param_loader.loadParam("beta_min", beta_min);
  param_loader.loadParam("dt", dt);
  param_loader.loadParam("initial_positions/" + _uav_name_ + "/x", _required_initial_position_[0]);
  param_loader.loadParam("initial_positions/" + _uav_name_ + "/y", _required_initial_position_[1]);
  param_loader.loadParam("initial_positions/" + _uav_name_ + "/z", _required_initial_position_[2]);
  param_loader.loadParam("monitored_area_origin/x", _monitored_area_origin_[0]);
  param_loader.loadParam("monitored_area_origin/y", _monitored_area_origin_[1]);
  param_loader.loadParam("monitored_area_origin/z", _monitored_area_origin_[2]);
  param_loader.loadParam("max_distance_to_initial_position", _dist_to_start_limit_);
  param_loader.loadParam("maximum_distance_conn", maximum_distance_conn);
  param_loader.loadParam("size_neighbors1", size_neighbors1);
  param_loader.loadParam("size_obstacles1", size_obstacles1);
  param_loader.loadParam("final_positions/" + _uav_name_ + "/x", destination[0]);
  param_loader.loadParam("final_positions/" + _uav_name_ + "/y", destination[1]);
  param_loader.loadParam("final_positions/" + _uav_name_ + "/z", destination[2]);
  param_loader.loadParam("Adj_matrix_" + _uav_name_, Adj_matrix);
  param_loader.loadParam("noisy_measurements", noisy_measurements);
  param_loader.loadParam("noisy_angle", noisy_angle);
  param_loader.loadParam("threshold", threshold);
  param_loader.loadParam("window_length", window_length);
  param_loader.loadParam("bias_error", bias_error);
  param_loader.loadParam("cwvd_rob", cwvd_rob);
  param_loader.loadParam("cwvd_obs", cwvd_obs);
  param_loader.loadParam("refZ", refZ_);
  param_loader.loadParam("controlled_dimensions", _c_dimensions_);
  //param_loader.loadParam("simulation", simulation_);
  param_loader.loadParam("sim", simulation_);
  param_loader.loadParam("min_z", min_z);
  param_loader.loadParam("max_z", max_z);
  param_loader.loadParam("flag_3D", flag_3D);
  param_loader.loadParam("use_z_rule", use_z_rule);

  param_loader.loadParam("use_livox_tilted", use_livox_tilted);
  param_loader.loadParam("livox_tilt_deg", livox_tilt_deg);
  param_loader.loadParam("livox_fov", livox_fov);
  param_loader.loadParam("searchRadius", searchRadius); 
  param_loader.loadParam("use_bonxai_mapping", use_bonxai_mapping);
  param_loader.loadParam("map_resolution", map_resolution);
  // param_loader.loadParam("use_voxel", use_voxel);
  double tmp;

  omp_set_num_threads(4); //TODO load? 

  livox_translation = Eigen::Vector3d(0.0, 0.0, 0.10); //TODO actually measure this on uav, and maybe loading it

  radius_sensing = radius / cwvd_obs + searchRadius;//radius is r_A, and r_sensing is for the points I need to consider only due to the agressivity
  save_scene_to_csv = false;

  param_loader.loadParam("max_obstacle_integration_dist", tmp);
  max_obstacle_integration_dist_sqr_ = pow(tmp, 2);

  // param_loader.loadParam("initial_positions/" + _uav_name_ + "/z", destination[2]);
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[RblController]: Could not load all parameters!");
    ros::shutdown();
  }
  //}

  _required_initial_position_ += _monitored_area_origin_;
  destination[0] += _monitored_area_origin_[0];
  destination[1] += _monitored_area_origin_[1];
  destination[2] += _monitored_area_origin_[2];
  size_neighbors.assign(_uav_names_.size() - 1, size_neighbors1);

  // erase this uav from the list of uavs
  auto it = std::find(_uav_names_.begin(), _uav_names_.end(), _uav_name_);

  if (it != _uav_names_.end()) {
    this_uav_idx_ = it - _uav_names_.begin();
    _uav_names_.erase(it);
    _uvdar_ids_.erase(_uvdar_ids_.begin() + this_uav_idx_);
  } else {
    ROS_ERROR("[RBLController]: This UAV is not part of the formation! Check the config file. Shutting down node.");
    ros::shutdown();
  }
  beta      = betaD;
  n_drones_ = _uav_names_.size();

  uav_neighbors_.resize(n_drones_);
  uav_positions_.resize(n_drones_);
  neighbors_and_obstacles_noisy.resize(n_drones_);

  goal[0]          = destination[0];
  goal[1]          = destination[1];
  goal[2]          = destination[2];
  goal_original[0] = destination[0];
  goal_original[1] = destination[1];
  goal_original[2] = destination[2];

  // std::vector<Eigen::Vector3d> uav_positionsN_(uav_positions_);
  /* create multiple subscribers to read uav odometries */
  // iterate through drones except this drone and target
  for (int i = 0; i < _uav_names_.size(); i++) {
    _uav_uvdar_ids_[_uvdar_ids_[i]] = i;
  } 
  uav_odom_subscriber_ = nh.subscribe("/" + _uav_name_ + "/estimation_manager/odom_main", 1, &RBLController::odomCallback, this);
  sub_uvdar_filtered_poses_.push_back(nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/" + _uav_name_ + "/uvdar/measuredPoses", 1, boost::bind(&RBLController::callbackNeighborsUsingUVDAR, this, _1)));
  mrs_lib::SubscribeHandlerOptions shopts;
  //
  shopts.nh                 = nh;
  shopts.node_name          = "RBLController";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_position_command_ = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");
  // clusters_sub_.push_back(nh.subscribe<visualization_msgs::MarkerArray>("/" + _uav_name_ + "/rplidar/clusters_", 1, &RBLController::clustersCallback, this));

  // clusters_sub_1.push_back(nh.subscribe<visualization_msgs::MarkerArray>("/" + _uav_name_ + "/rplidar/clusters_1", 1, &RBLController::clustersCallback1, this));

  // waypoints_sub_.push_back(nh.subscribe<visualization_msgs::MarkerArray>("/" + _uav_name_ + "/replanner/waypoints_", 1, &RBLController::waypointsCallback, this));
  // waypoint_sub = nh.subscribe<geometry_msgs::PoseArray>("/" + _uav_name_ + "/octomap_planner/waypoints", 1, &RBLController::waypointsCallback, this);
    waypoint_sub = nh.subscribe<visualization_msgs::MarkerArray>("/" + _uav_name_ + "/trajectory_generation/markers/final", 1, &RBLController::markerCallback, this);
    
  // initialize timers
  timer_set_reference_ = nh.createTimer(ros::Rate(_rate_timer_set_reference_), &RBLController::callbackTimerSetReference, this);
  timer_set_active_wp_ = nh.createTimer(ros::Rate(_rate_timer_set_reference_), &RBLController::goalUpdateLoop, this);
  timer_diagnostics_   = nh.createTimer(ros::Rate(_rate_timer_diagnostics_), &RBLController::callbackTimerDiagnostics, this);
  // timer_pub_ = nh.createTimer(ros::Rate(_rate_timer_set_reference_), &RBLController::callbackPublisher, this);

  // initialize service servers
  service_activate_control_   = nh.advertiseService("control_activation_in", &RBLController::activationServiceCallback, this);
  service_deactivate_control_ = nh.advertiseService("control_deactivation_in", &RBLController::deactivationServiceCallback, this);
  service_fly_to_start_       = nh.advertiseService("fly_to_start_in", &RBLController::flyToStartServiceCallback, this);
  service_save_to_csv_        = nh.advertiseService("save_to_csv_in", &RBLController::saveToCsvServiceCallback, this);

  // initialize service clients
  sc_set_position_  = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("ref_pos_out");
  sc_goto_position_ = nh.serviceClient<mrs_msgs::Vec4>("goto_out");

  // initialize publishers
  pub_destination_    = nh.advertise<visualization_msgs::Marker>("destination_out", 1, true);
  pub_position_       = nh.advertise<visualization_msgs::Marker>("position_out", 1, true);
  pub_centroid_       = nh.advertise<visualization_msgs::Marker>("centroid_out", 1, true);
  pub_pointCloud_     = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);
  pub_cellA_          = nh.advertise<sensor_msgs::PointCloud2>("cell_A", 1, true);
  pub_cell_sensed_A_  = nh.advertise<sensor_msgs::PointCloud2>("actively_sensed_A", 1, true);
  pub_planes_         = nh.advertise<visualization_msgs::MarkerArray>("planes", 1, true);
  pub_norms_          = nh.advertise<visualization_msgs::MarkerArray>("planes_norms", 1, true);
  pub_path_           = nh.advertise<nav_msgs::Path>("path", 1, true);

  if (simulation_) {
    // sub_pointCloud2_  = nh.subscribe("/" + _uav_name_ + "/livox_fake_scan", 1, &RBLController::pointCloud2Callback, this);
    sub_pointCloud2_  = nh.subscribe("/" + _uav_name_ + "/octomap_local_vis/octomap_point_cloud_centers", 1, &RBLController::pointCloud2Callback, this);
  } else { 
    if (use_bonxai_mapping) {
      // sub_pointCloud2_  = nh.subscribe("/" + _uav_name_ + "/bonxai_server/local_pc", 1, &RBLController::pointCloud2Callback, this);
      sub_pointCloud2_  = nh.subscribe("/" + _uav_name_ + "/octomap_local_vis/octomap_point_cloud_centers", 1, &RBLController::pointCloud2Callback, this);
    } else {
      sub_pointCloud2_  = nh.subscribe("/" + _uav_name_ + "/pcl_filter/livox_points_processed", 1, &RBLController::pointCloud2Callback, this);
    }
  }

  //TODO del
  sub_velocity_ = nh.subscribe("/" + _uav_name_ + "/hw_api/velocity", 1, &RBLController::velocityCallback, this);

  // initialize transformer
  transformer_ = std::make_shared<mrs_lib::Transformer>(nh, "RBLController");
  transformer_->retryLookupNewest(true);

  is_initialized_ = true;
  ROS_INFO("[RBLController]: Initialization completed.");
}
//}

/*Publisher for Visualization //{ */



/*RBLController::publishNorms () //{ */
void RBLController::publishPath(const std::vector<Eigen::Vector3d>& path) {
  if (path.empty()) {
    ROS_WARN("publishPath: Received empty path.");
    return;
  }

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = _control_frame_;

  for (const auto& pt : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _control_frame_;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = pt.z();

    // Optional: Set orientation to identity
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path_msg.poses.push_back(pose);
  }

  pub_path_.publish(path_msg);
}
//}

/*RBLController::publishNorms () //{ */
void RBLController::publishNorms(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& planes) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(planes.size());

  for (size_t i = 0; i < planes.size(); ++i) {
      const Eigen::Vector3d& normal = planes[i].first;
      const Eigen::Vector3d& point = planes[i].second;

      visualization_msgs::Marker& marker = marker_array.markers[i];
      marker.header.frame_id = _control_frame_; // Use the provided frame ID
      marker.header.stamp = ros::Time::now();
      marker.ns = "plane_norms";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::ARROW;

      // Set marker scale (length and thickness of the arrow)
      marker.scale.x = 0.1; // is the shaft diameter
      marker.scale.y = 0.15; //is the head diameter.
      marker.scale.z = 0.2; // is not zero, it specifies the head length.

      // Set marker color
      marker.color.r = 1.0f; // Red
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f; // Opaque

      // Set the end point of the arrow (vector direction)
      marker.pose.orientation.w = 1;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;

      marker.points.resize(2);
      marker.points[0].x = point.x();
      marker.points[0].y = point.y();
      marker.points[0].z = point.z();

      marker.points[1].x = point.x() + 3*normal.x();
      marker.points[1].y = point.y() + 3*normal.y();
      marker.points[1].z = point.z() + 3*normal.z();

      marker.lifetime = ros::Duration(); // Infinite lifetime
  }

  pub_norms_.publish(marker_array);
}
//}

/*RBLController::publishPlanes () //{ */
void RBLController::publishPlanes(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& planes) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(planes.size());

  for (size_t i = 0; i < planes.size(); ++i) {
    const Eigen::Vector3d& normal = planes[i].first;
    const Eigen::Vector3d& point = planes[i].second;
    visualization_msgs::Marker& marker = marker_array.markers[i];
    marker.header.frame_id = _control_frame_; // Adjust frame ID as needed
    marker.header.stamp = ros::Time::now();
    marker.ns = "planes";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::Marker::CUBE; // Use CUBE for a rectangle/square

    // Calculate the orientation of the plane (rotation)
    Eigen::Vector3d up(0, 1, 0); // Default up vector
    Eigen::Vector3d z_axis = normal.normalized();
    Eigen::Vector3d x_axis = up.cross(z_axis).normalized();
    if (x_axis.norm() < 1e-6) { // Handle case where normal is parallel to up
      x_axis = Eigen::Vector3d(1, 0, 0);
    }
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;

    Eigen::Quaterniond quaternion(rotation);

    // Set marker pose
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    // Set marker scale (size of the rectangle/square)
    marker.scale.x = 7.0; // Adjust size as needed
    marker.scale.y = 7.0;
    marker.scale.z = 0.01; // Thin plane
    
    // Set marker color (semi-transparent)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f; // Transparency

    marker.lifetime = ros::Duration(); // Infinite lifetime
  }

  pub_planes_.publish(marker_array);
}
//}

/*RBLController::publishCellA () //{ */
void RBLController::publishCellA(std::vector<Eigen::Vector3d> points) {
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pclCloud.width = points.size();
  pclCloud.height = 1;
  pclCloud.is_dense = true;
  pclCloud.points.resize(pclCloud.width * pclCloud.height);

  for (size_t i = 0; i < points.size(); ++i) {
    pclCloud.points[i].x = static_cast<float>(points[i].x());
    pclCloud.points[i].y = static_cast<float>(points[i].y());
    pclCloud.points[i].z = static_cast<float>(points[i].z());
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(pclCloud, cloud_msg);
  cloud_msg.header.frame_id = _control_frame_;
  cloud_msg.header.stamp = ros::Time::now();

  pub_cellA_.publish(cloud_msg);
}
//}

/*RBLController::publishCellA () //{ */
void RBLController::publishCellActivelySensedA(std::vector<Eigen::Vector3d> points) {
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pclCloud.width = points.size();
  pclCloud.height = 1;
  pclCloud.is_dense = true;
  pclCloud.points.resize(pclCloud.width * pclCloud.height);

  for (size_t i = 0; i < points.size(); ++i) {
    pclCloud.points[i].x = static_cast<float>(points[i].x());
    pclCloud.points[i].y = static_cast<float>(points[i].y());
    pclCloud.points[i].z = static_cast<float>(points[i].z());
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(pclCloud, cloud_msg);
  cloud_msg.header.frame_id = _control_frame_;
  cloud_msg.header.stamp = ros::Time::now();

  pub_cell_sensed_A_.publish(cloud_msg);
}
//}

/*RBLController::publishPosition() //{ */
void RBLController::publishPosition() {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = _control_frame_;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "position";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = robot_pos[0];
  marker.pose.position.y    = robot_pos[1];
  marker.pose.position.z    = robot_pos[2];
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 2 * encumbrance;  // Adjust size as necessary
  marker.scale.y            = 2 * encumbrance;
  marker.scale.z            = 2 * encumbrance;
  marker.color.r            = 0.0;  // Red color
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;  // Fully opaque

  pub_position_.publish(marker);
}
//}

/*RBLController::publishCentroid() //{ */
void RBLController::publishCentroid() {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = _control_frame_;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "centroid";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = c1_to_rviz[0];
  marker.pose.position.y    = c1_to_rviz[1];
  marker.pose.position.z    = c1_to_rviz[2];  // Assuming obstacles are on the ground
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.2;  // Adjust size as necessary
  marker.scale.y            = 0.2;
  marker.scale.z            = 0.2;
  marker.color.r            = 1.0;  // Red color
  marker.color.g            = 0.0;
  marker.color.b            = 0.0;
  marker.color.a            = 1.0;  // Fully opaque

  pub_centroid_.publish(marker);
}
//}

/*RBLController::publishDestination() //{ */
void RBLController::publishDestination() {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = _control_frame_;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "destination";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = goal[0];
  marker.pose.position.y    = goal[1];
  marker.pose.position.z    = goal[2];  
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 4 * encumbrance;  // Adjust size as necessary
  marker.scale.y            = 4 * encumbrance;
  marker.scale.z            = 4 * encumbrance;
  marker.color.r            = 0.0;  // Red color
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;  // Fully opaque

  pub_destination_.publish(marker);
}
//}

/*RBLController::publishPcl() //{ */
void RBLController::publishPcl() {
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(processed_cloud, output);
  output.header.frame_id = _control_frame_;
  output.header.stamp = ros::Time::now();
  pub_pointCloud_.publish(output);

}
//}




/*RBLController::functions() //{ */

std::vector<Eigen::Vector3d> RBLController::points_inside_circle(Eigen::Vector3d robot_pos, double radius, double step_size) { 
  double x_center = robot_pos[0];
  double y_center = robot_pos[1];
  
  int    x_min    = static_cast<int>((x_center - radius) / step_size);
  int    x_max    = static_cast<int>((x_center + radius) / step_size);
  int    y_min    = static_cast<int>((y_center - radius) / step_size);
  int    y_max    = static_cast<int>((y_center + radius) / step_size);

  std::vector<double> x_coords, y_coords;
  for (int i = x_min; i <= x_max; ++i)
    x_coords.push_back(i * step_size);
  for (int j = y_min; j <= y_max; ++j)
    y_coords.push_back(j * step_size);

  std::vector<Eigen::Vector3d> points;
  for (auto x : x_coords) {
    for (auto y : y_coords) {
      double distance = std::sqrt(std::pow((x - x_center), 2) + std::pow((y - y_center), 2));
      if (distance <= radius)
        points.push_back(Eigen::Vector3d(x, y, refZ_));
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> RBLController::boundary_points_sphere(Eigen::Vector3d robot_pos, double radius, double step_size){
  double x_center = robot_pos[0];
  double y_center = robot_pos[1];
  double z_center = robot_pos[2];

  int    x_min_idx    = static_cast<int>(std::floor((x_center - radius) / step_size));
  int    x_max_idx    = static_cast<int>(std::ceil((x_center + radius) / step_size));
  int    y_min_idx    = static_cast<int>(std::floor((y_center - radius) / step_size));
  int    y_max_idx    = static_cast<int>(std::ceil((y_center + radius) / step_size));
  int    z_min_idx    = static_cast<int>(std::floor((z_center - radius) / step_size));
  int    z_max_idx    = static_cast<int>(std::ceil((z_center + radius) / step_size));

  std::vector<Eigen::Vector3d> boundary_points;

  for (int i = x_min_idx; i <= x_max_idx; ++i) {
    for (int j = y_min_idx; j <= y_max_idx; ++j) {
      for (int k = z_min_idx; k <= z_max_idx; ++k) {
        double x = i * step_size;
        double y = j * step_size;
        double z = k * step_size;

        double distance_sq = std::pow(x - x_center, 2) + std::pow(y - y_center, 2) + std::pow(z - z_center, 2);

        double epsilon = step_size * 0.5;

        if (std::abs(distance_sq - std::pow(radius, 2)) <= epsilon * radius) {
             boundary_points.push_back(Eigen::Vector3d(x, y, z));
        }
      }
    }
  }

  return boundary_points;
}

std::vector<Eigen::Vector3d> RBLController::project_boundary_points_on_encumbrance(Eigen::Vector3d robot_pos, double encumbrance, std::vector<Eigen::Vector3d> boundary_points) {
  std::vector<Eigen::Vector3d> projected_points;
  for (int i = 0; i < boundary_points.size(); ++i) {
    Eigen::Vector3d point = boundary_points[i];
    Eigen::Vector3d projected_point = encumbrance * (point - robot_pos)/(point - robot_pos).norm() + robot_pos;
    projected_points.push_back(projected_point);
  }
  return projected_points;
}


std::vector<Eigen::Vector3d> RBLController::points_inside_sphere(Eigen::Vector3d robot_pos, double radius, double step_size) {
  double x_center = robot_pos[0];
  double y_center = robot_pos[1];
  double z_center = robot_pos[2];

  int    x_min    = static_cast<int>((x_center - radius) / step_size);
  int    x_max    = static_cast<int>((x_center + radius) / step_size);
  int    y_min    = static_cast<int>((y_center - radius) / step_size);
  int    y_max    = static_cast<int>((y_center + radius) / step_size);
  int    z_min    = static_cast<int>((z_center - radius) / step_size);
  int    z_max    = static_cast<int>((z_center + radius) / step_size);

  std::vector<double> x_coords, y_coords, z_coords;
  for (int i = x_min; i <= x_max; ++i)
    x_coords.push_back(i * step_size);
  for (int j = y_min; j <= y_max; ++j)
    y_coords.push_back(j * step_size);
  for (int k = z_min; k <= z_max; ++k)
    z_coords.push_back(k * step_size);

  std::vector<Eigen::Vector3d> points;
  for (auto x : x_coords) {
    for (auto y : y_coords) {
      for (auto z : z_coords) {
        double distance = std::sqrt(std::pow((x - x_center), 2) + std::pow((y - y_center), 2) + std::pow((z - z_center), 2));
        if (distance <= radius && z >= min_z && z <= max_z)
          points.push_back(Eigen::Vector3d(x, y, z));
      }
    }
  }
  return points;
}


std::vector<Eigen::Vector3d> RBLController::insert_vec3d_at_index(const std::vector<Eigen::Vector3d> &vec, size_t idx, const Eigen::Vector3d &value) {
  if (idx > vec.size()) {
    idx = vec.size();
  }
  std::vector<Eigen::Vector3d> new_vec = vec;
  new_vec.insert(new_vec.begin() + idx, value);
  
  return new_vec;
}

std::vector<Eigen::Vector3d> RBLController::fixed_neighbors(const std::vector<Eigen::Vector3d> &positions, const std::vector<int> &adjacency_matrix, size_t my_index) {
  std::vector<Eigen::Vector3d> neighbors_filtered;

  // Iterate over the row of the adjacency matrix corresponding to my_index
  for (size_t j = 0; j < adjacency_matrix.size(); ++j) {
    // Check if the value is 1, indicating a neighbor
    if (adjacency_matrix[j] == 1) {
      // Add the position of the neighbor to neighbors_filtered
      neighbors_filtered.push_back(Eigen::Vector3d{positions[j][0], positions[j][1], positions[j][2]});
    }
  }
  return neighbors_filtered;
}

std::vector<Eigen::Vector3d> RBLController::communication_constraint(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &neighbors) {
  new_neighbors.clear();
  // Initialize neighbors_past_measurements if empty
  if (neighbors_past_measurements.size() < neighbors.size()) {
    neighbors_past_measurements.resize(neighbors.size());
  }

  for (size_t j = 0; j < neighbors.size(); ++j) {
    const auto &neighbor = neighbors[j];

    // Add current neighbor to past measurements
    if (neighbors_past_measurements[j].size() >= 10) {
      neighbors_past_measurements[j].erase(neighbors_past_measurements[j].begin());
    }
    neighbors_past_measurements[j].push_back(Eigen::Vector3d(neighbor[0], neighbor[1], neighbor[2]));

    // Calculate mean of past measurements
    double mean_x = std::accumulate(neighbors_past_measurements[j].begin(), neighbors_past_measurements[j].end(), 0.0,
                                    [](double sum, const Eigen::Vector3d &p) { return sum + p[0]; }) /
                    neighbors_past_measurements[j].size();
    double mean_y = std::accumulate(neighbors_past_measurements[j].begin(), neighbors_past_measurements[j].end(), 0.0,
                                    [](double sum, const Eigen::Vector3d &p) { return sum + p[1]; }) /
                    neighbors_past_measurements[j].size();
    double mean_z = std::accumulate(neighbors_past_measurements[j].begin(), neighbors_past_measurements[j].end(), 0.0,
                                    [](double sum, const Eigen::Vector3d &p) { return sum + p[2]; }) /
                    neighbors_past_measurements[j].size();

    // Check distance from robot_pos to neighbor mean position
    double dx            = mean_x - robot_pos[0];
    double dy            = mean_y - robot_pos[1];
    double dz            = mean_z - robot_pos[2]; 
    double distance_3D    = std::sqrt(dx * dx + dy * dy + dz * dz);
    double bearing_angle = std::atan2(dy, dx);
    if (distance_3D > maximum_distance_conn) {

      Eigen::Vector3d direction(dx, dy, dz);
      direction.normalize();

      Eigen::Vector3d new_pos = robot_pos + (maximum_distance_conn - threshold) * direction;

      mean_x = new_pos[0];
      mean_y = new_pos[1];
      mean_z = new_pos[2];
    }

    new_neighbors.push_back(Eigen::Vector3d(mean_x, mean_y, mean_z));
  }


  std::vector<int>                          index;
  std::vector<Eigen::Vector3d> new_points = points;

  for (const auto &neighbor : new_neighbors) {
    for (size_t i = 0; i < new_points.size(); ++i) {
      double distance = std::sqrt(std::pow(new_points[i][0] - neighbor[0], 2) + std::pow(new_points[i][1] - neighbor[1], 2) + std::pow(new_points[i][2] - neighbor[2], 2));

      if (distance > maximum_distance_conn) {
        index.push_back(i);
      }
    }
  }
  std::vector<Eigen::Vector3d> filtered_points;
  for (size_t i = 0; i < new_points.size(); ++i) {
    if (std::find(index.begin(), index.end(), i) == index.end()) {
      filtered_points.push_back(new_points[i]);
    }
  }
  return filtered_points;
}

Eigen::Vector3d RBLController::closest_point_from_voxel(Eigen::Vector3d robot_pos, Eigen::Vector3d voxel_center, double map_resolution) {
  Eigen::Vector3d half_resolution(map_resolution / 2.0, map_resolution / 2.0, map_resolution / 2.0);

  Eigen::Vector3d min_corner = voxel_center - half_resolution;
  Eigen::Vector3d max_corner = voxel_center + half_resolution;

  Eigen::Vector3d closest_point;
  closest_point.x() = std::clamp(robot_pos.x(), min_corner.x(), max_corner.x());
  closest_point.y() = std::clamp(robot_pos.y(), min_corner.y(), max_corner.y());
  closest_point.z() = std::clamp(robot_pos.z(), min_corner.z(), max_corner.z());

  return closest_point;
}

std::vector<Eigen::Vector3d> RBLController::find_closest_points_using_voxel_fast( const Eigen::Vector3d                         &robot_pos,
                                                                                  const std::vector<Eigen::Vector3d>            &points,
                                                                                  const std::vector<Eigen::Vector3d>            &boundary_cell_A_points,
                                                                                  const std::vector<Eigen::Vector3d>            &neighbors,
                                                                                  pcl::PointCloud<pcl::PointXYZ>                cloud) {

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<bool> remove_mask(points.size(), false);

  std::vector<Eigen::Vector3d> plane_normals;
  std::vector<Eigen::Vector3d> plane_points;

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> planes;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> closest_plane;
  Eigen::Vector3d far_plane = Eigen::Vector3d(1000.0, 0.0, 0.0);
  closest_plane.first = far_plane;
  closest_plane.second = far_plane;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  if (cloud.size() > 0) {
    kdtree.setInputCloud(cloud.makeShared());
  }

  // Query KD-tree for nearest neighbors
  std::vector<int> k_indices(1);
  std::vector<float> k_sqr_distances(1);

  std::vector<int> radius_indices;
  std::vector<float> radius_sqr_distances;

  //check other agents
  for (const auto &neighbor : neighbors) {
    // std::pair<Eigen::Vector3d, Eigen::Vector3d> plane;
    double Delta_i_j = 2*encumbrance; //TODO redo this if encum is different
    Eigen::Vector3d tilde_p_i = Delta_i_j * (neighbor - robot_pos)/( (neighbor - robot_pos).norm() ) + robot_pos;
    Eigen::Vector3d tilde_p_j = Delta_i_j * (robot_pos - neighbor)/( (robot_pos - neighbor).norm() ) + neighbor;
    Eigen::Vector3d plane_norm = tilde_p_j - tilde_p_i;
    Eigen::Vector3d plane_point = tilde_p_i + cwvd_rob * plane_norm;

    if ((robot_pos - tilde_p_i).norm() <= (robot_pos - tilde_p_j).norm()) {
      plane_normals.push_back(plane_norm);
      plane_points.push_back(plane_point);
    } else {
      plane_normals.push_back(-plane_norm);
      plane_points.push_back(plane_point);
    }
  }


  // #pragma omp parallel for schedule(static)
  for (int i = 0; i < static_cast<int>(boundary_cell_A_points.size()); ++i) {
    if (remove_mask[i]) {
      continue;
    }

    Eigen::Vector3d point = boundary_cell_A_points[i];
    
    // Eigen::Vector3d point_to_robot = (point - robot_pos);
    // double dist_to_robot = (point - robot_pos).norm();

    if (!remove_mask[i] && cloud.size() > 0) {
      pcl::PointXYZ searchPoint(point[0], point[1], point[2]);
      Eigen::Vector3d voxel_to_check;
      Eigen::Vector3d dirVec = point - robot_pos;
      bool found_valid_voxel = false;

      if (kdtree.radiusSearch(searchPoint, radius_sensing, radius_indices, radius_sqr_distances) > 0) {
        for (size_t j = 0; j< radius_indices.size(); ++j) {
          const auto& current_voxel = cloud.points[radius_indices[j]];
          voxel_to_check = Eigen::Vector3d(current_voxel.x, current_voxel.y, current_voxel.z);
          if (dirVec.dot(voxel_to_check) > 0){
            found_valid_voxel = true;
            break;
          }
        }
        if (!found_valid_voxel) {
          continue;
        }
      // if (kdtree.nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances) > 0) {
        // int nearestVoxelIndex = k_indices[0];
        // const auto& voxel = cloud.points[k_indices[0]];
        // Eigen::Vector3d voxel_point(voxel[0], voxel[1], voxel[2]); //center point of voxel
        Eigen::Vector3d voxel_point = voxel_to_check;
        // voxel_point[0] = voxel.x;
        // voxel_point[1] = voxel.y;
        // voxel_point[2] = voxel.z;
        Eigen::Vector3d closest_p_on_vox = closest_point_from_voxel(robot_pos, voxel_point, map_resolution); //closest point on the voxel
        // double Delta_i_j = encumbrance + (voxel_point - closest_p_on_vox).norm();
        double Delta_i_j = encumbrance + sqrt(3*pow(map_resolution/2.0, 2));
        Eigen::Vector3d tilde_p_i = Delta_i_j * (voxel_point - robot_pos)/(voxel_point - robot_pos).norm() + robot_pos;
        Eigen::Vector3d tilde_p_j = Delta_i_j * (robot_pos - voxel_point)/(robot_pos - voxel_point).norm() + voxel_point;

        Eigen::Vector3d plane_norm, plane_point;
        if ((robot_pos - tilde_p_i).norm() <= (robot_pos - tilde_p_j).norm()) {
          plane_norm = tilde_p_j - tilde_p_i;
          plane_point = tilde_p_i + cwvd_obs * plane_norm;
        } else {
          plane_norm = tilde_p_i - tilde_p_j;
          plane_point = tilde_p_j;
        }
        plane_normals.push_back(plane_norm);
        plane_points.push_back(plane_point);
        
        if ((closest_plane.second - robot_pos).norm() >= (plane_point - robot_pos).norm() ) {
          closest_plane.first = plane_norm;
          closest_plane.second = plane_point;
        }

      } else {
        remove_mask[i] = true;
      }
    }
  }
  // auto start_time = std::chrono::high_resolution_clock::now();
  planes.push_back(closest_plane);
  publishPlanes(planes);
  // publishNorms(plane_normals);

  // Precompute plane offset values
  std::vector<double> plane_offsets(plane_normals.size());

  #pragma omp parallel for
  for (int j = 0; j < static_cast<int>(plane_normals.size()); ++j) {
    plane_offsets[j] = plane_normals[j].dot(plane_points[j]);
  }


  #pragma omp parallel for schedule(dynamic, 64)
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (remove_mask[i]) continue;

    const Eigen::Vector3d& point = points[i];
    bool should_remove = false;

    for (size_t j = 0; j < plane_normals.size(); ++j) {
      double side = plane_normals[j].dot(point) - plane_offsets[j];
      if (side >= 0) {
        should_remove = true;
        break;
      }
    }

    if (should_remove) remove_mask[i] = true;
  }

  std::vector<Eigen::Vector3d> result_points;
  for ( size_t i = 0; i < points.size(); ++i) {
    if (!remove_mask[i]) {
      result_points.push_back(points[i]);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now(); // End timing
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // Calculate duration
  // std::cout << "1 Fast partitioning of the cell A based on voxels and neighbors took: " << duration.count() << " milliseconds." << std::endl;

  return result_points;
}

std::vector<Eigen::Vector3d> RBLController::find_closest_points_using_voxel_faster( const Eigen::Vector3d                         &robot_pos,
                                                                                  const std::vector<Eigen::Vector3d>            &points,
                                                                                  const std::vector<Eigen::Vector3d>            &boundary_cell_A_points,
                                                                                  const std::vector<Eigen::Vector3d>            &neighbors,
                                                                                  pcl::PointCloud<pcl::PointXYZ>                cloud) {

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<bool> remove_mask(points.size(), false);

  std::vector<Eigen::Vector3d> plane_normals;
  std::vector<Eigen::Vector3d> plane_points;

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> planes;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> closest_plane;
  Eigen::Vector3d far_plane = Eigen::Vector3d(1000.0, 0.0, 0.0);
  closest_plane.first = far_plane;
  closest_plane.second = far_plane;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  if (cloud.size() > 0) {
    kdtree.setInputCloud(cloud.makeShared());
  }

  // Query KD-tree for nearest neighbors
  std::vector<int> k_indices(1);
  std::vector<float> k_sqr_distances(1);

  std::vector<int> radius_indices;
  std::vector<float> radius_sqr_distances;

  //check other agents
  for (const auto &neighbor : neighbors) {
    // std::pair<Eigen::Vector3d, Eigen::Vector3d> plane;
    double Delta_i_j = 2*encumbrance; //TODO redo this if encum is different
    Eigen::Vector3d tilde_p_i = Delta_i_j * (neighbor - robot_pos)/( (neighbor - robot_pos).norm() ) + robot_pos;
    Eigen::Vector3d tilde_p_j = Delta_i_j * (robot_pos - neighbor)/( (robot_pos - neighbor).norm() ) + neighbor;
    Eigen::Vector3d plane_norm = tilde_p_j - tilde_p_i;
    Eigen::Vector3d plane_point = tilde_p_i + cwvd_rob * plane_norm;

    if ((robot_pos - tilde_p_i).norm() <= (robot_pos - tilde_p_j).norm()) {
      plane_normals.push_back(plane_norm);
      plane_points.push_back(plane_point);
    } else {
      plane_normals.push_back(-plane_norm);
      plane_points.push_back(plane_point);
    }
  }

  pcl::PointXYZ searchPoint(robot_pos[0], robot_pos[1], robot_pos[2]);
  if (kdtree.radiusSearch(searchPoint, radius_sensing, radius_indices, radius_sqr_distances) > 0) {
    for (size_t i = 0; i < radius_indices.size(); ++i) {
      const auto& current_voxel = cloud.points[radius_indices[i]];
    
      Eigen::Vector3d voxel_point(current_voxel.x, current_voxel.y, current_voxel.z);
      Eigen::Vector3d closest_p_on_vox = closest_point_from_voxel(robot_pos, voxel_point, map_resolution);
      double Delta_i_j = encumbrance + sqrt(3*pow(map_resolution/2.0, 2));
      Eigen::Vector3d tilde_p_i = Delta_i_j * (voxel_point - robot_pos)/(voxel_point - robot_pos).norm() + robot_pos;
      Eigen::Vector3d tilde_p_j = Delta_i_j * (robot_pos - voxel_point)/(robot_pos - voxel_point).norm() + voxel_point;

      Eigen::Vector3d plane_norm, plane_point;
      if ((robot_pos - tilde_p_i).norm() <= (robot_pos - tilde_p_j).norm()) {
        plane_norm = tilde_p_j - tilde_p_i;
        plane_point = tilde_p_i + cwvd_obs * plane_norm;
      } else {
        plane_norm = tilde_p_i - tilde_p_j;
        plane_point = tilde_p_j;
      }
      plane_normals.push_back(plane_norm);
      plane_points.push_back(plane_point);
      
      if ((closest_plane.second - robot_pos).norm() >= (plane_point - robot_pos).norm() ) {
        closest_plane.first = plane_norm;
        closest_plane.second = plane_point;
      }
    }
  }

  // auto start_time = std::chrono::high_resolution_clock::now();
  planes.push_back(closest_plane);
  publishPlanes(planes);
  // publishNorms(plane_normals);

  // Precompute plane offset values
  std::vector<double> plane_offsets(plane_normals.size());

  #pragma omp parallel for
  for (int j = 0; j < static_cast<int>(plane_normals.size()); ++j) {
    plane_offsets[j] = plane_normals[j].dot(plane_points[j]);
  }


  #pragma omp parallel for schedule(dynamic, 64)
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (remove_mask[i]) continue;

    const Eigen::Vector3d& point = points[i];
    bool should_remove = false;

    for (size_t j = 0; j < plane_normals.size(); ++j) {
      double side = plane_normals[j].dot(point) - plane_offsets[j];
      if (side >= 0) {
        should_remove = true;
        break;
      }
    }

    if (should_remove) remove_mask[i] = true;
  }

  std::vector<Eigen::Vector3d> result_points;
  for ( size_t i = 0; i < points.size(); ++i) {
    if (!remove_mask[i]) {
      result_points.push_back(points[i]);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now(); // End timing
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // Calculate duration
  // std::cout << "2 Fastest partitioning of the cell A based on voxels and neighbors took: " << duration.count() << " milliseconds." << std::endl;

  return result_points;
}

std::vector<Eigen::Vector3d> RBLController::find_closest_points_using_voxel(const Eigen::Vector3d            &robot_pos,
                                                                const std::vector<Eigen::Vector3d>           &points,
                                                                const std::vector<Eigen::Vector3d>           &neighbors,
                                                                pcl::PointCloud<pcl::PointXYZ>  cloud) {

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<bool> remove_mask(points.size(), false);

  std::vector<Eigen::Vector3d> plane_normals;
  std::vector<Eigen::Vector3d> plane_points;

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> planes;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  if (cloud.size() > 0) {
    kdtree.setInputCloud(cloud.makeShared());
  }

  // Query KD-tree for nearest neighbors
  std::vector<int> k_indices(1);
  std::vector<float> k_sqr_distances(1);

  //check other agents
  for (const auto &neighbor : neighbors) {
    // std::pair<Eigen::Vector3d, Eigen::Vector3d> plane;
    double Delta_i_j = 2*encumbrance; //TODO redo this if encum is different
    Eigen::Vector3d tilde_p_i = Delta_i_j * (neighbor - robot_pos)/( (neighbor - robot_pos).norm() ) + robot_pos;
    Eigen::Vector3d tilde_p_j = Delta_i_j * (robot_pos - neighbor)/( (robot_pos - neighbor).norm() ) + neighbor;
    Eigen::Vector3d plane_norm = tilde_p_j - tilde_p_i;
    Eigen::Vector3d plane_point = tilde_p_i + cwvd_rob * plane_norm;

    if ((robot_pos - tilde_p_i).norm() <= (robot_pos - tilde_p_j).norm()) {
      plane_normals.push_back(plane_norm);
      plane_points.push_back(plane_point);
    } else {
      plane_normals.push_back(-plane_norm);
      plane_points.push_back(plane_point);
    }
  }


  // #pragma omp parallel for schedule(static)
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (remove_mask[i]) {
      continue;
    }

    Eigen::Vector3d point = points[i];
    
    Eigen::Vector3d point_to_robot = (point - robot_pos);
    double dist_to_robot = (point - robot_pos).norm();

    if (!remove_mask[i] && cloud.size() > 0) {
      pcl::PointXYZ searchPoint;
      searchPoint.x = point[0];
      searchPoint.y = point[1];
      searchPoint.z = point[2];

      if (kdtree.nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances) > 0) {
        int nearestVoxelIndex = k_indices[0];
        const auto& voxel = cloud.points[nearestVoxelIndex];
        Eigen::Vector3d voxel_point; //center point of voxel
        voxel_point[0] = voxel.x;
        voxel_point[1] = voxel.y;
        voxel_point[2] = voxel.z;
        Eigen::Vector3d closest_p_on_vox = closest_point_from_voxel(robot_pos, voxel_point, map_resolution); //closest point on the voxel
        double Delta_i_j = encumbrance + (voxel_point - closest_p_on_vox).norm();
        Eigen::Vector3d tilde_p_i = Delta_i_j * (voxel_point - robot_pos)/(voxel_point - robot_pos).norm() + robot_pos;
        Eigen::Vector3d tilde_p_j = Delta_i_j * (robot_pos - voxel_point)/(robot_pos - voxel_point).norm() + voxel_point;

        Eigen::Vector3d plane_norm, plane_point;
        if ((robot_pos - tilde_p_i).norm() <= (robot_pos - tilde_p_j).norm()) {
          plane_norm = tilde_p_j - tilde_p_i;
          plane_point = tilde_p_i + cwvd_obs * plane_norm;;
        } else {
          plane_norm = tilde_p_i - tilde_p_j;
          plane_point = tilde_p_j + cwvd_obs * plane_norm;;
        }
        plane_normals.push_back(plane_norm);
        plane_points.push_back(plane_point);
        
        std::pair<Eigen::Vector3d, Eigen::Vector3d> plane;
        plane.first = plane_norm;
        plane.second = plane_point; 
        planes.push_back(plane);
      } else {
        remove_mask[i] = true;
      }
    }
  }
  // auto start_time = std::chrono::high_resolution_clock::now();

  publishPlanes(planes);
  // publishNorms(plane_normals);

  // Precompute plane offset values
  std::vector<double> plane_offsets(plane_normals.size());

  #pragma omp parallel for
  for (int j = 0; j < static_cast<int>(plane_normals.size()); ++j) {
    plane_offsets[j] = plane_normals[j].dot(plane_points[j]);
  }


  #pragma omp parallel for schedule(dynamic, 64)
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (remove_mask[i]) continue;

    const Eigen::Vector3d& point = points[i];
    bool should_remove = false;

    // Manual unrolling or early exit on hot planes
    for (size_t j = 0; j < plane_normals.size(); ++j) {
      double side = plane_normals[j].dot(point) - plane_offsets[j];
      if (side >= 0) {
        should_remove = true;
        break;
      }
    }

    if (should_remove) remove_mask[i] = true;
  }

  std::vector<Eigen::Vector3d> result_points;
  for ( size_t i = 0; i < points.size(); ++i) {
    if (!remove_mask[i]) {
      result_points.push_back(points[i]);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now(); // End timing
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // Calculate duration
  std::cout << "Partitioning of the cell A based on voxels and neighbors took: " << duration.count() << " milliseconds." << std::endl;

  return result_points;
}

std::vector<Eigen::Vector3d> RBLController::find_closest_points(const Eigen::Vector3d                        &robot_pos,
                                                                const std::vector<Eigen::Vector3d>           &points,
                                                                const std::vector<Eigen::Vector3d>           &neighbors,
                                                                const std::vector<double>                    &only_robots,
                                                                const pcl::PolygonMesh                       &mesh) {

  std::vector<bool> remove_mask(points.size(), false);
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> planes; //norm, point
  // std::cout << "Points before triangle computation: " << points.size() << " ." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  
  auto start_time = std::chrono::high_resolution_clock::now();

  //use kdtree for faster computation of distances from triangles
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZ>);
  if (mesh.polygons.size()>0){    
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_xyz);
    for (const auto& polygon : mesh.polygons) {
      Eigen::Vector3f v1(cloud_xyz->points[polygon.vertices[0]].x, cloud_xyz->points[polygon.vertices[0]].y, cloud_xyz->points[polygon.vertices[0]].z);
      Eigen::Vector3f v2(cloud_xyz->points[polygon.vertices[1]].x, cloud_xyz->points[polygon.vertices[1]].y, cloud_xyz->points[polygon.vertices[1]].z);
      Eigen::Vector3f v3(cloud_xyz->points[polygon.vertices[2]].x, cloud_xyz->points[polygon.vertices[2]].y, cloud_xyz->points[polygon.vertices[2]].z);

      pcl::PointXYZ centroid;
      centroid.x = (v1[0] + v2[0] + v3[0]) / 3.0f;
      centroid.y = (v1[1] + v2[1] + v3[1]) / 3.0f;
      centroid.z = (v1[2] + v2[2] + v3[2]) / 3.0f;
      centroids->push_back(centroid);
    }
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  if (centroids->size() > 0) {
    kdtree.setInputCloud(centroids);
  }


  // Query KD-tree for nearest neighbors
  std::vector<int> k_indices(1);
  std::vector<float> k_sqr_distances(1);


  for (size_t i = 0; i < points.size(); ++i) {
    if (remove_mask[i]) {
      continue;
    }
    
    Eigen::Vector3d point = points[i];
    //TODO have a look at the point it doesnt seem smart
    Eigen::Vector3d point_to_robot = (point - robot_pos);
    double dist_to_robot = (point - robot_pos).norm();

    //check other agents
    for (const auto &neighbor : neighbors) {
      double alpha = std::atan2(neighbor[1] - robot_pos[1], neighbor[0] - robot_pos[0]); //azimuth
      double beta = std::atan2(neighbor[2] - robot_pos[2], std::sqrt(std::pow(neighbor[0] - robot_pos[0], 2) + std::pow(neighbor[1] - robot_pos[1], 2))); //elevation
      Eigen::Vector3d d_xyz(std::cos(beta)*std::cos(alpha), std::cos(beta)*std::sin(alpha), std::sin(beta));

      if (d_xyz.dot(point_to_robot)> cwvd_rob * (robot_pos - neighbor).norm()) {
        remove_mask[i] = true;
        break;
      }
    }

    //check polygon mesh
    if (!remove_mask[i] && centroids->size() > 0) {
      pcl::PointXYZ searchPoint;
      searchPoint.x = point[0];
      searchPoint.y = point[1];
      searchPoint.z = point[2];

      if (kdtree.nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances) > 0) {
        int nearestPolygonIndex = k_indices[0];
        const auto& polygon = mesh.polygons[nearestPolygonIndex];

        Eigen::Vector3d v1(cloud_xyz->points[polygon.vertices[0]].x, cloud_xyz->points[polygon.vertices[0]].y, cloud_xyz->points[polygon.vertices[0]].z);
        Eigen::Vector3d v2(cloud_xyz->points[polygon.vertices[1]].x, cloud_xyz->points[polygon.vertices[1]].y, cloud_xyz->points[polygon.vertices[1]].z);
        Eigen::Vector3d v3(cloud_xyz->points[polygon.vertices[2]].x, cloud_xyz->points[polygon.vertices[2]].y, cloud_xyz->points[polygon.vertices[2]].z);
        Eigen::Vector3d p(point[0], point[1], point[2]);

        Eigen::Vector3d closest_point_triangle = closest_point_from_triangle(p, v1, v2, v3);
        // Eigen::Vector3d p_d = p.cast<double>();
        double dist_robot_to_triangle = (robot_pos - closest_point_triangle).norm();

        double alpha_tri = std::atan2(closest_point_triangle[1] - robot_pos[1], closest_point_triangle[0] - robot_pos[0]); //azimuth
        double beta_tri = std::atan2(closest_point_triangle[2] - robot_pos[2], std::sqrt(std::pow(closest_point_triangle[0] - robot_pos[0], 2) + std::pow(closest_point_triangle[1] - robot_pos[1], 2))); //elevation
        Eigen::Vector3d d_xyz_tri(std::cos(beta_tri)*std::cos(alpha_tri), std::cos(beta_tri)*std::sin(alpha_tri), std::sin(beta_tri));

        if (d_xyz_tri.dot(point_to_robot) > cwvd_obs * dist_robot_to_triangle) {
          std::pair<Eigen::Vector3d, Eigen::Vector3d> plane;
          Eigen::Vector3d normal = closest_point_triangle - robot_pos;
          plane.first = normal; //normal pointing away from uav
          plane.second = robot_pos + cwvd_obs * normal; //point on the plane
          planes.push_back(plane);
        }
      } else {
        remove_mask[i] = true;
      }
    }
  }

  //publishPlanes(planes);
  //publishNorms(planes);

  // Precompute plane offset values
  std::vector<double> plane_offsets(planes.size());
  for (size_t j = 0; j < planes.size(); ++j) {
    plane_offsets[j] = planes[j].first.dot(planes[j].second);
  }


  for (size_t i = 0; i < points.size(); ++i) {
    if (remove_mask[i]){
      continue; //point already removed - move to another
    }
    const Eigen::Vector3d& point = points[i];
    for (size_t j = 0; j < planes.size(); ++j) {
      const Eigen::Vector3d& normal = planes[j].first;

      // Compute side test efficiently
      double side = normal.dot(point) - plane_offsets[j];
      if (side >= 0){
        remove_mask[i] = true;
        break;
      }
    }
  }



  std::vector<Eigen::Vector3d> result_points;
  for ( size_t i = 0; i < points.size(); ++i) {
    if (!remove_mask[i]) {
      result_points.push_back(points[i]);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now(); // End timing

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // Calculate duration

  //std::cout << "Partitioning of the cell A based on triangels took: " << duration.count() << " milliseconds." << std::endl;
  //std::cout << "Closer points before filter by planes: " << points.size() << " ." << std::endl;

  //std::cout << "After filter by planes: " << result_points.size() << " ." << std::endl;

  return result_points;
}

std::vector<Eigen::Vector3d> RBLController::pointCloudToEigenVector(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.reserve(cloud.size());

  for (const auto& pt : cloud.points) {
    eigen_points.emplace_back(pt.x, pt.y, pt.z);
  }

  return eigen_points;
}
double RBLController::point_triangle_distance(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  //https://stackoverflow.com/questions/2924795/fastest-way-to-compute-point-to-triangle-distance-in-3d
  const Eigen::Vector3f ab = b - a;
  const Eigen::Vector3f ac = c - a;
  const Eigen::Vector3f ap = p - a;

  const float d1 = ab.dot(ap);
  const float d2 = ac.dot(ap);

  if (d1 <= 0.f && d2 <= 0.f) return ap.norm(); //#1

  const Eigen::Vector3f bp = p - b;
  const float d3 = ab.dot(bp);
  const float d4 = ac.dot(bp);
  if (d3 >= 0.f && d4 <= d3) return bp.norm(); //#2

  const Eigen::Vector3f cp = p - c;
  const float d5 = ab.dot(cp);
  const float d6 = ac.dot(cp);
  if (d6 >= 0.f && d5 <= d6) return cp.norm(); //#3

  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
    const float v = d1 / (d1 - d3);
    return (p - (a + v * ab)).norm(); //#4
  }

  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
    const float v = d2 / (d2 - d6);
    return (p - (a + v * ac)).norm(); //#5
  }
    
  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
    const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return (p - (b + v * (c - b))).norm(); //#6
  }

  const float denom = 1.f / (va + vb + vc);
  const float v = vb * denom;
  const float w = vc * denom;
  return (p - (a + v * ab + w * ac)).norm(); //#0
}

Eigen::Vector3d RBLController::closest_point_from_triangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
  //https://stackoverflow.com/questions/2924795/fastest-way-to-compute-point-to-triangle-distance-in-3d
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = p - a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);

  if (d1 <= 0.0 && d2 <= 0.0) return a; //#1

  const Eigen::Vector3d bp = p - b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) return b; //#2

  const Eigen::Vector3d cp = p - c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) return c; //#3

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return (a + v * ab); //#4
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double v = d2 / (d2 - d6);
    return (a + v * ac); //#5
  }
    
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return (b + v * (c - b)); //#6
  }

  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  return (a + v * ab + w * ac); //#0
}


std::vector<double> RBLController::compute_scalar_value(const std::vector<double> &x_test, 
                                                        const std::vector<double> &y_test, 
                                                        const std::vector<double> &z_test,
                                                        const Eigen::Vector3d &destination, double beta) {
  std::vector<double> scalar_values;
  for (size_t i = 0; i < x_test.size(); ++i) {
    double distance     = std::sqrt(std::pow((x_test[i] - destination[0]), 2) + std::pow((y_test[i] - destination[1]), 2) + std::pow((z_test[i] - destination[2]), 2));
    double scalar_value = std::exp(-distance / beta);
    scalar_values.push_back(scalar_value);
  }
  return scalar_values;
}

Eigen::Matrix3d RBLController::Rx(double angle) {
  Eigen::Matrix3d Rx;
  Rx << 1, 0, 0,
     0, std::cos(angle), -std::sin(angle),
     0, std::sin(angle), std::cos(angle);
  return Rx;
}

Eigen::Matrix3d RBLController::Ry(double angle) {
  Eigen::Matrix3d Ry;
  Ry << std::cos(angle), 0, std::sin(angle),
        0, 1, 0,
        -std::sin(angle), 0, std::cos(angle);
  return Ry;
}

Eigen::Matrix3d RBLController::Rz(double angle) {
  Eigen::Matrix3d Rz;
  Rz << std::cos(angle), -std::sin(angle), 0,
        std::sin(angle), std::cos(angle), 0,
        0, 0, 1;
  return Rz;
}

std::vector<Eigen::Vector3d> RBLController::slice_sphere(std::vector<Eigen::Vector3d> points, Eigen::Vector3d robot_pos, double angle_deg, double livox_fov, Eigen::Vector3d roll_pitch_yaw) {
  double livox_tilt_rad = angle_deg * M_PI / 180.0;
  double livox_fov_rad = livox_fov * M_PI / 180.0;
  Eigen::Vector3d livox_pos = Eigen::Vector3d(robot_pos[0] + livox_translation[0], robot_pos[1] + livox_translation[1], robot_pos[2] + livox_translation[2]);

  Eigen::Matrix3d R_rpy = Rx(roll_pitch_yaw[0]) * Ry(roll_pitch_yaw[1]) * Rz(roll_pitch_yaw[2]); 
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> planes;

  std::pair<Eigen::Vector3d, Eigen::Vector3d> plane;
  Eigen::Vector3d norm = Ry(livox_tilt_rad) * Eigen::Vector3d(0,0,1);
  norm = R_rpy * norm;
  //std::cout << "Norm x: " << norm[0] << ", y: " << norm[1] << ", z: " << norm[2] << std::endl;
  plane.first = norm;
  plane.second = livox_pos;
  planes.push_back(plane);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> plane_upper;
  Eigen::Vector3d norm_upper = Ry(livox_tilt_rad-livox_fov_rad) * Eigen::Vector3d(0,0,1);
  norm_upper = R_rpy * norm_upper;
  plane_upper.first = norm_upper;
  plane_upper.second = livox_pos; 
  planes.push_back(plane_upper);
  // publishPlanes(planes);
  // publishNorms(planes);

  points.erase(
    std::remove_if(points.begin(), points.end(), [&](const Eigen::Vector3d& point) {
      double res = norm.dot(point - livox_pos);
      double res_upper = norm_upper.dot(point - livox_pos);
      return res < 0 || res_upper > 0;
    }),
    points.end()
  );

  return points;
}

void RBLController::goalUpdateLoop(const ros::TimerEvent&) {

    std::vector<geometry_msgs::Point> points_copy;

    {
        std::lock_guard<std::mutex> lock(points_mutex_);
        if (dense_points_.empty()) return;
        points_copy = dense_points_;
    }

    // Step 1: Find the closest point on the path to the current robot position
    size_t start_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points_copy.size(); ++i) {
        double dx = points_copy[i].x - robot_pos[0];
        double dy = points_copy[i].y - robot_pos[1];
        double dz = points_copy[i].z - robot_pos[2];
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < min_dist) {
            min_dist = dist;
            start_idx = i;
        }
    }

    // Step 2: Walk forward along the path and accumulate distance
    const double target_distance = 3.0;
    double accumulated_distance = 0.0;
    size_t best_idx = start_idx;

    for (size_t i = start_idx; i < points_copy.size() - 1; ++i) {
        const geometry_msgs::Point& p1 = points_copy[i];
        const geometry_msgs::Point& p2 = points_copy[i + 1];
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        double segment = std::sqrt(dx * dx + dy * dy + dz * dz);
        accumulated_distance += segment;

        if (accumulated_distance >= target_distance) {
            best_idx = i + 1;
            break;
        }
    }

    const geometry_msgs::Point& chosen = points_copy[best_idx];
    destination[0] = goal[0] = chosen.x;
    destination[1] = goal[1] = chosen.y;
    destination[2] = goal[2] = chosen.z;

    ROS_DEBUG("Goal updated at path distance ~4m: idx=%lu, x=%.2f y=%.2f z=%.2f",
              best_idx, chosen.x, chosen.y, chosen.z);
}

// void RBLController::goalUpdateLoop(const ros::TimerEvent&) {

//     std::vector<geometry_msgs::Point> points_copy;

//     std::cout << "ciao1  " << std::endl;
//     {
//         std::lock_guard<std::mutex> lock(points_mutex_);
//         std::cout << "Dense points size: " << dense_points_.size() << std::endl;
//         if (dense_points_.empty()) return;
//         std::cout << "ciao " << std::endl;
//         points_copy = dense_points_;
//     }

//     // Same logic: find point closest to 4 meters
//     const double target_distance = 4.0;
//     size_t best_idx = 0;
//     double min_diff = std::numeric_limits<double>::max();

//     for (size_t i = 0; i < points_copy.size(); ++i) {
//         const geometry_msgs::Point& pt = points_copy[i];
//         double dx = pt.x - robot_pos[0];
//         double dy = pt.y - robot_pos[1];
//         double dz = pt.z - robot_pos[2];
//         double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
//         double diff = std::abs(dist - target_distance);

//         if (diff < min_diff) {
//             min_diff = diff;
//             best_idx = i;
//         }
//     }

//     const geometry_msgs::Point& chosen = points_copy[best_idx];
//     destination[0] = goal[0] = chosen.x;
//     destination[1] = goal[1] = chosen.y;
//     destination[2] = goal[2] = chosen.z;

//     ROS_DEBUG("High-freq goal update: x=%.2f y=%.2f z=%.2f", chosen.x, chosen.y, chosen.z);
// }

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> RBLController::get_centroid(
    Eigen::Vector3d robot_pos, double radius, double step_size, std::vector<Eigen::Vector3d> &neighbors,
    const std::vector<double> &size_neighbors, std::vector<Eigen::Vector3d> &neighbors_and_obstacles,
    const std::vector<double> &size_neighbors_and_obstacles, double encumbrance, const Eigen::Vector3d &destination, double &beta,
    std::vector<Eigen::Vector3d> &neighbors_and_obstacles_noisy) {

  neighbors_and_obstacles_noisy.clear();
  neighbors_and_obstacles_noisy = neighbors_and_obstacles;
  std::vector<Eigen::Vector3d> cell_A_points;
  std::vector<Eigen::Vector3d> boundary_cell_A_points;
  std::vector<Eigen::Vector3d> projected_boundry_A_points;
  if (flag_3D){
    cell_A_points = points_inside_sphere(robot_pos, radius, step_size);
    boundary_cell_A_points = boundary_points_sphere(robot_pos, radius, step_size);
    projected_boundry_A_points = project_boundary_points_on_encumbrance(robot_pos, encumbrance, boundary_cell_A_points);
  } else {
    cell_A_points = points_inside_circle(robot_pos, radius, step_size);
    projected_boundry_A_points = cell_A_points;
  }
  
  std::vector<Eigen::Vector3d> voronoi_circle_intersection;
  std::vector<Eigen::Vector3d> voronoi_circle_intersection_connectivity;

  if (!neighbors_and_obstacles_noisy.empty() || processed_cloud.size() > 0) {
    // if (use_voxel) {
    // voronoi_circle_intersection = find_closest_points_using_voxel(robot_pos, cell_A_points, neighbors_and_obstacles_noisy, processed_cloud);
    // std::cout << "1 voronoi slow: " << voronoi_circle_intersection.size() << std::endl;
    // voronoi_circle_intersection = find_closest_points_using_voxel_fast(robot_pos, cell_A_points, projected_boundry_A_points, neighbors_and_obstacles_noisy, processed_cloud);
    voronoi_circle_intersection = find_closest_points_using_voxel_faster(robot_pos, cell_A_points, projected_boundry_A_points, neighbors_and_obstacles_noisy, processed_cloud);
    // std::cout << "2 voronoi fast: " << voronoi_circle_intersection.size() << std::endl;
    // } else {
      // voronoi_circle_intersection = find_closest_points(robot_pos, cell_A_points, neighbors_and_obstacles_noisy, size_neighbors, mesh);
    // }

    //TODO do fixed neighbors for 3d
    // all_uavs = insert_vec3d_at_index(neighbors, this_uav_idx_, Eigen::Vector3d{val.first, val.second, 1000.0}); //TODO val as third argument
    
    // fixed_neighbors_vec = fixed_neighbors(all_uavs, Adj_matrix, this_uav_idx_);

    // voronoi_circle_intersection_connectivity = communication_constraint(voronoi_circle_intersection, fixed_neighbors_vec);
    voronoi_circle_intersection_connectivity = voronoi_circle_intersection;

    if (voronoi_circle_intersection_connectivity.empty()){
      std::cout << "WARNING" << std::endl;
      voronoi_circle_intersection_connectivity.push_back(Eigen::Vector3d(robot_pos[0], robot_pos[1], robot_pos[2]));
    }
    if (voronoi_circle_intersection.empty()) {
      std::cout << "WARNING de luxe" << std::endl;
      voronoi_circle_intersection.push_back(Eigen::Vector3d(robot_pos[0], robot_pos[1], robot_pos[2]));
    }
  } else {
    voronoi_circle_intersection_connectivity = cell_A_points;
    voronoi_circle_intersection = cell_A_points;
  }

  std::vector<double> x_in, y_in, z_in;
  for (const auto &point : voronoi_circle_intersection_connectivity) {
    x_in.push_back(point[0]);
    y_in.push_back(point[1]);
    z_in.push_back(point[2]);
  }

  std::vector<double> x_in_no_neigh, y_in_no_neigh, z_in_no_neigh;
  for (const auto &point : cell_A_points) {
    x_in_no_neigh.push_back(point[0]);
    y_in_no_neigh.push_back(point[1]);
    z_in_no_neigh.push_back(point[2]);
  }

  std::vector<double> x_in_no_conn, y_in_no_conn, z_in_no_conn;
  for (const auto &point : voronoi_circle_intersection) {
    x_in_no_conn.push_back(point[0]);
    y_in_no_conn.push_back(point[1]);
    z_in_no_conn.push_back(point[2]);
  }

  // Compute scalar values
  std::vector<double> scalar_values          = compute_scalar_value(x_in, y_in, z_in, destination, beta);
  std::vector<double> scalar_values_no_neigh = compute_scalar_value(x_in_no_neigh, y_in_no_neigh, z_in_no_neigh, destination, beta);
  std::vector<double> scalar_values_no_conn  = compute_scalar_value(x_in_no_conn, y_in_no_conn, z_in_no_conn, destination, beta);

  // Compute the weighted centroid
  double sum_x_in_times_scalar_values = 0.0;
  double sum_y_in_times_scalar_values = 0.0;
  double sum_z_in_times_scalar_values = 0.0;
  double sum_scalar_values            = 0.0;

  for (size_t i = 0; i < x_in.size(); ++i) {
    sum_x_in_times_scalar_values += x_in[i] * scalar_values[i];
    sum_y_in_times_scalar_values += y_in[i] * scalar_values[i];
    sum_z_in_times_scalar_values += z_in[i] * scalar_values[i];
    sum_scalar_values += scalar_values[i];
  }

  Eigen::Vector3d centroid(sum_x_in_times_scalar_values / sum_scalar_values, sum_y_in_times_scalar_values / sum_scalar_values, sum_z_in_times_scalar_values / sum_scalar_values);

  // Compute the centroid without neighbors
  double sum_x_in_no_neigh_times_scalar_values = 0.0;
  double sum_y_in_no_neigh_times_scalar_values = 0.0;
  double sum_z_in_no_neigh_times_scalar_values = 0.0;
  double sum_scalar_values_no_neigh            = 0.0;

  for (size_t i = 0; i < x_in_no_neigh.size(); ++i) {
    sum_x_in_no_neigh_times_scalar_values += x_in_no_neigh[i] * scalar_values_no_neigh[i];
    sum_y_in_no_neigh_times_scalar_values += y_in_no_neigh[i] * scalar_values_no_neigh[i];
    sum_z_in_no_neigh_times_scalar_values += z_in_no_neigh[i] * scalar_values_no_neigh[i];
    sum_scalar_values_no_neigh += scalar_values_no_neigh[i];
  }

  Eigen::Vector3d centroid_no_neigh(sum_x_in_no_neigh_times_scalar_values / sum_scalar_values_no_neigh, sum_y_in_no_neigh_times_scalar_values / sum_scalar_values_no_neigh, sum_z_in_no_neigh_times_scalar_values / sum_scalar_values_no_neigh);

  // Compute the centroid without conn
  double sum_x_in_no_conn_times_scalar_values = 0.0;
  double sum_y_in_no_conn_times_scalar_values = 0.0;
  double sum_z_in_no_conn_times_scalar_values = 0.0;
  double sum_scalar_values_no_conn            = 0.0;

  for (size_t i = 0; i < x_in_no_conn.size(); ++i) {
    sum_x_in_no_conn_times_scalar_values += x_in_no_conn[i] * scalar_values_no_conn[i];
    sum_y_in_no_conn_times_scalar_values += y_in_no_conn[i] * scalar_values_no_conn[i];
    sum_z_in_no_conn_times_scalar_values += z_in_no_conn[i] * scalar_values_no_conn[i];
    sum_scalar_values_no_conn += scalar_values_no_conn[i];
  }

  Eigen::Vector3d centroid_no_conn(sum_x_in_no_conn_times_scalar_values / sum_scalar_values_no_conn, sum_y_in_no_conn_times_scalar_values / sum_scalar_values_no_conn, sum_z_in_no_conn_times_scalar_values / sum_scalar_values_no_conn);

  if (use_livox_tilted) {
    std::vector<Eigen::Vector3d> legal_centroid_position = slice_sphere(voronoi_circle_intersection, robot_pos, livox_tilt_deg, livox_fov, roll_pitch_yaw);
    if (legal_centroid_position.size() > 0){
      publishCellA(voronoi_circle_intersection);
      // publishCellA(boundary_cell_A_points);
    }
    if (legal_centroid_position.size() > 0) {
      publishCellActivelySensedA(legal_centroid_position);
    }
    // centroid = closest_in_legal_set(centroid, legal_centroid_position, robot_pos, step_size);
    // centroid_no_neigh = closest_in_legal_set(centroid_no_neigh, legal_centroid_position, robot_pos, step_size);
    // centroid_no_conn = closest_in_legal_set(centroid_no_conn, legal_centroid_position, robot_pos, step_size);
  } else {
    if (voronoi_circle_intersection.size() > 0){
      publishCellA(voronoi_circle_intersection);
    }
  }

  return std::make_tuple(centroid, centroid_no_neigh, centroid_no_conn);
}

Eigen::Vector3d RBLController::closest_in_legal_set(Eigen::Vector3d c_pos, std::vector<Eigen::Vector3d> legal_set, Eigen::Vector3d robot_pos, double step_size) {
  if (legal_set.empty()) {
    std::cerr << "Error: Legal set is empty!" << std::endl;
    return robot_pos; 
  }

  Eigen::Vector3d closest_point = legal_set[0]; // Initialize with the first point
  double min_distance = (c_pos - legal_set[0]).norm(); // Initial distance

  for (const auto& point : legal_set) {
    double distance = (c_pos - point).norm();
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = point;
    }
  }

  if (min_distance > step_size) {
    return closest_point;
  }else {
    return c_pos;
  }
}


void RBLController::apply_rules(double &beta, const std::vector<double> &c1, const std::vector<double> &c2, const std::vector<double> &current_position,
                                double dt, double beta_min, const double &betaD, std::vector<double> &goal, double d1, double &th, double d2, double d3,
                                double d4, double d5, double d6, double d7, Eigen::Vector3d &destination, std::vector<double> &c1_no_rotation) {

  // Extract x, y, z components from current_position
  double current_j_x = current_position[0];
  double current_j_y = current_position[1];
  double current_j_z = current_position[2];

  if (flag_3D) {
    // first condition
    double dist_c1_c2 = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2) + pow((c1[2] - c2[2]), 2));
    double dist_current_c1 = sqrt(pow(current_j_x - c1[0], 2) + pow(current_j_y - c1[1], 2) + pow(current_j_z - c1[2], 2));
    if (dist_c1_c2 > d2 && dist_current_c1 < d1) {
      beta = std::max(beta - dt, beta_min); //  std::max(beta - dt * (epsilon), beta_min);
    } else {
      beta = beta - dt * (beta - betaD);
    }

    //azimuth
    // second condition
    double dist_c1_c2_plane_xy = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2));
    double dist_current_c1_plane_xy = sqrt(pow(current_j_x - c1[0], 2) + pow(current_j_y - c1[1], 2));
    bool dist_c1_c2_plane_xy_d4 = dist_c1_c2_plane_xy > d4;
    if (dist_c1_c2_plane_xy_d4 && dist_current_c1_plane_xy < d3) {
      // th = std::min(th + 0.1 * dt, M_PI / 2);
      th = std::min(th + dt, M_PI / 2);
    } else {
      th = std::max(0.0, th - 2*dt);
    }


    // third condition
    if (th == M_PI / 2 && sqrt(pow((current_j_x - c1_no_rotation[0]), 2) + pow((current_j_y - c1_no_rotation[1]), 2)) > dist_current_c1_plane_xy) {
      th = 0;
    }
    // th = 0;  
    // std::cout << "theta: " << th << std::endl;
    //elevation
    double dist_c1_c2_z = fabs(c1[2] - c2[2]);
    double dist_current_c1_z = fabs(current_j_z - c1[2]);
    double to_c2_z = c2[2] - current_j_z;

    double dist_current_c2_plane_xy = sqrt(pow(current_j_x - c2[0] ,2) + pow(current_j_y - c2[1] ,2));

    if (use_z_rule) {
      if (dist_c1_c2_z > d5 && dist_current_c1_z < d6 || (dist_current_c2_plane_xy - dist_current_c1_plane_xy) > d7) { //modify phi up down based on some if
        double direction_to_goal = std::atan2(goal[0] - current_j_x, goal[1] - current_j_y); // [0, 2pi)
        if (direction_to_goal < 0){
          direction_to_goal += 2 * M_PI;
        }
        // double direction_influence = direction_to_goal < M_PI ? 1.0 : -1.0;
        //linear mapping
        double direction_influence = (direction_to_goal / M_PI) - 1.0;

        double w1 = 0.7; // Weight for to_c2_z TODO LOAD but this ok
        double w2 = 0.3; // Weight for delta_c1_z

        // Calculate weighted average
        double combined_influence = (w1 * to_c2_z + w2 * direction_influence) / (w1 + w2);
        
        if (combined_influence > 0) { 
          ph = std::min(M_PI_4, ph + dt);
        } else { 
          ph = std::max(-M_PI_4, ph - dt);
        }
        // std::cout << "modifying phi: " << ph << std::endl;
      } else { //converge back to ph = 0
        if (ph > 0.0) {         
          ph = std::max(0.0, ph - dt);
        } else if (ph < 0.0) { 
          ph = std::min(0.0, ph + dt);
        } else {
          ph = 0.0;
        }
        // std::cout << "converging back: " << ph << std::endl;
      }

      //check if any vertical avoidance && if if vertical adjust makes drone further from uav/obstacle
      double dist_current_c2_z = fabs(current_j_z - c1[2]);
      if (fabs(ph) == M_PI_4 && dist_current_c2_z > fabs(current_j_z - c1_no_rotation[2])) {
        ph = 0;  
        // std::cout << "Phi reset: " << ph << std::endl;
      }
    }


    double dx = goal[0] - current_j_x;
    double dy = goal[1] - current_j_y;
    double dz = goal[2] - current_j_z;
    double dist = sqrt(dx * dx + dy * dy + dz* dz);

    double theta = atan2(dy, dx); //!! azimuthal angle in xy plane
    double phi = acos(dz / dist); //polar angel from the z axis

    double new_theta = theta - th;
    double new_phi = phi + ph;

    destination[0] = current_j_x + dist * sin(new_phi) * cos(new_theta);
    destination[1] = current_j_y + dist * sin(new_phi) * sin(new_theta);
    destination[2] = current_j_z + dist * cos(new_phi);
    
    
  } else {
    // first condition
    double dist_c1_c2 = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2));
    if (dist_c1_c2 > d2 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d1) {
      beta = std::max(beta - dt, beta_min);
    } else {
      beta = beta - dt * (beta - betaD);
    }

    // std::cout <<  "distp_c1 = " << sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) << std::endl;

    // second condition
    bool dist_c1_c2_d4 = dist_c1_c2 > d4;
    if (dist_c1_c2_d4 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d3) {
      th = std::min(th + dt, M_PI / 2);
      /* std::cout << "RHSrule" << std::endl; */
    } else {
      th = std::max(0.0, th - dt);
    }

    // third condition
    if (th == M_PI / 2 && sqrt(pow((current_j_x - c1_no_rotation[0]), 2) + pow((current_j_y - c1_no_rotation[1]), 2)) >
                              sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2))) {
      th = 0;
      // std::cout << "reset" << std::endl;
    }
    /* std::cout << "theta : " << th << ", beta: " << beta << std::endl; */
    // Compute the angle and new position
    double angle        = atan2(goal[1] - current_j_y, goal[0] - current_j_x);
    double new_angle    = angle - th;
    double distance     = sqrt(pow((goal[0] - current_j_x), 2) + pow((goal[1] - current_j_y), 2));
    destination[0]     = current_j_x + distance * cos(new_angle);
    destination[1]     = current_j_y + distance * sin(new_angle);

  }
}
//}


/* odomCallback() //{ */

void RBLController::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

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

  mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_position, uav_position_);
  mrs_lib::set_mutexed(mutex_uav_odoms_, euler, roll_pitch_yaw);

}

//}

/* clustersCallback() //{ */
// void RBLController::clustersCallback(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg) {
//   // Clear the previous list of obstacles

//   std::scoped_lock lock(mutex_obstacles_);
//   if (simulation_) {
//     obstacles_.clear();
//   }

//   // Iterate through markers to compute centroids
//   for (const auto &marker : marker_array_msg->markers) {
//     Eigen::Vector3d closest_point;
//     double                    min_distance = std::numeric_limits<double>::max();
//     if (marker.type == visualization_msgs::Marker::POINTS) {
//       for (const auto &point : marker.points) {
//         geometry_msgs::PointStamped point_transformed;
//         point_transformed.header  = marker.header;
//         point_transformed.point.x = point.x;
//         point_transformed.point.y = point.y;
//         point_transformed.point.z = point.z;

//         // Transform the point to the control frame
//         auto res = transformer_->transformSingle(point_transformed, _control_frame_);
//         if (res) {
//           point_transformed = res.value();
//         } else {
//           ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform obstacle point to control frame.");
      
//           return;
//         }

//         // Compute distance to UAV position
//         double dx       = point_transformed.point.x - uav_position_[0];
//         double dy       = point_transformed.point.y - uav_position_[1];
//         double dz       = point_transformed.point.z - uav_position_[2];
//         double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

//         // Check if this is the closest point so far
//         if (distance < min_distance) {
//           min_distance  = distance;
//           closest_point = Eigen::Vector3d(point_transformed.point.x, point_transformed.point.y, point_transformed.point.z);
//         }
//       }
//       // Store the point in obstacles vector
//       if (min_distance < 15.0) {
//         obstacles_.emplace_back(closest_point[0], closest_point[1], closest_point[2]);
//       }
//     }
//   }
// }

// //}

// /* clustersCallback1() //{ */
// void RBLController::clustersCallback1(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg) {
//   // Clear the previous list of obstacles
//   std::scoped_lock lock(mutex_obstacles_);
//   if (simulation_) {
//     obstacles_.clear();
//   }

//   // Iterate through markers
//   for (const auto &marker : marker_array_msg->markers) {
//     Eigen::Vector3d closest_point;
//     double                    min_distance = std::numeric_limits<double>::max();
//     if (marker.type == visualization_msgs::Marker::POINTS) {
//       // Process all points in this marker
//       for (const auto &point : marker.points) {
//         geometry_msgs::PointStamped point_transformed;
//         point_transformed.header  = marker.header;
//         point_transformed.point.x = point.x;
//         point_transformed.point.y = point.y;
//         point_transformed.point.z = point.z;

//         // Transform the point to the control frame
//         auto res = transformer_->transformSingle(point_transformed, _control_frame_);
//         if (res) {
//           point_transformed = res.value();
//         } else {
//           ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform obstacle point to control frame.");
//           return;
//         }

//         // Compute distance to UAV position
//         double dx       = point_transformed.point.x - uav_position_[0];
//         double dy       = point_transformed.point.y - uav_position_[1];
//         double dz       = point_transformed.point.z - uav_position_[2];
//         double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

//         // Check if this is the closest point so far
//         if (distance < min_distance) {
//           min_distance  = distance;
//           closest_point = Eigen::Vector3d(point_transformed.point.x, point_transformed.point.y, point_transformed.point.z);
//         }
//       }
//       // Store the point in obstacles vector
//       if (min_distance < 15.0) {
//         obstacles_.emplace_back(closest_point[0], closest_point[1], closest_point[2]);
//       }
//     }
//   }
// }
/* //} */

void RBLController::markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    if (!msg->markers.empty() && msg->markers[0].points.size() > 1) {

        std::lock_guard<std::mutex> lock(points_mutex_);
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

        ROS_INFO("Interpolated to %lu total points", dense_points.size());
        // const geometry_msgs::Point& chosen = dense_points[best_idx];
        dense_points_ = dense_points;

        // Find point closest to exactly 4m from robot_pos
        const double target_distance = 4.0;
        size_t best_idx = 0;
        double min_diff = std::numeric_limits<double>::max();

        for (size_t i = 0; i < dense_points.size(); ++i) {
            const geometry_msgs::Point& pt = dense_points[i];
            double dx = pt.x - robot_pos[0];
            double dy = pt.y - robot_pos[1];
            double dz = pt.z - robot_pos[2];
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            double diff = std::abs(dist - target_distance);

            if (diff < min_diff) {
                min_diff = diff;
                best_idx = i;
            }
        }
    } else {
        ROS_WARN("Not enough points to interpolate.");
    }
}

/*RBLController::callbackNeighborsUsingUVDAR() //{ */
void RBLController::callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &array_poses) {

  // ROS_INFO("Received pose from uvdar");
  /* uav_neighbors_.assign(n_drones_ + 1, Eigen::Vector3d(1000, 0, 0)); */
  largest_eigenvalue_.assign(n_drones_ + 1, 0);
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

    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res) {
      new_point = res.value();
    } else {
      ROS_ERROR_THROTTLE(3.0, "[RBLController]: UVDAR: Could not transform positions to control frame.");
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
    has_this_pose_ = true;
    if (largest_eigenvalue < 15.0) {
      mrs_lib::set_mutexed(mutex_uav_uvdar_, transformed_position, uav_neighbors_[_uav_uvdar_ids_[uav_id]]);
      mrs_lib::set_mutexed(mutex_uav_uvdar_, largest_eigenvalue, largest_eigenvalue_[_uav_uvdar_ids_[uav_id]]);
    }
  }
}
//}

/* callbackTimerPubNeighbors() //{ */

void RBLController::callbackTimerPubNeighbors([[maybe_unused]] const ros::TimerEvent &event) {
  if (!is_initialized_ || !has_this_pose_) {
    return;
  }
}


//}


/* callbackTimerSetReference() Callback where we set the p_ref //{ */
void RBLController::callbackTimerSetReference([[maybe_unused]] const ros::TimerEvent &te) {
  if (!is_initialized_) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  if (!control_allowed_) {
    ROS_WARN_THROTTLE(3.0, "[RBLController]: Waiting for activation.");
    return;
  }

  mrs_msgs::Reference p_ref;

  {
    std::scoped_lock                       lock(mutex_uav_odoms_, mutex_position_command_, mutex_uav_uvdar_);
    auto                                   start = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector3d> neighbors;
    std::vector<Eigen::Vector3d> neighbors_and_obstacles;
    if (flag_3D){
      robot_pos = {uav_position_[0], uav_position_[1], uav_position_[2]};
    } else {
      robot_pos = {uav_position_[0], uav_position_[1], refZ_};
    }

    double distance2neigh;
    for (int j = 0; j < n_drones_; ++j) {
      if (flag_3D) {
        neighbors.push_back(Eigen::Vector3d{uav_neighbors_[j][0], uav_neighbors_[j][1], uav_neighbors_[j][2]});
        neighbors_and_obstacles.push_back(Eigen::Vector3d{uav_neighbors_[j][0], uav_neighbors_[j][1], uav_neighbors_[j][2]});
      } else {
        neighbors.push_back(Eigen::Vector3d{uav_neighbors_[j][0], uav_neighbors_[j][1], refZ_});
        neighbors_and_obstacles.push_back(Eigen::Vector3d{uav_neighbors_[j][0], uav_neighbors_[j][1], refZ_});
      }
    }

    { 
      size_neighbors_and_obstacles.clear();
      size_obstacles.assign(obstacles_.size(), size_obstacles1);
    }
    size_neighbors_and_obstacles = size_neighbors;  // Copy vec1 to vec3
    size_neighbors_and_obstacles.insert(size_neighbors_and_obstacles.end(), size_obstacles.begin(), size_obstacles.end());

    active_wp = destination;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.makeShared();
    if (cloud_ptr && !cloud_ptr->empty()) { 
      if (save_scene_to_csv) {
        std::cout << "saving raw point cloud to csv" << std::endl;
        savePCL2TOCSV(cloud_ptr, "raw_pointcloud.csv");
      }

      if (use_bonxai_mapping) {
        processed_cloud = *cloud_ptr;
      } else {
        processed_cloud = voxelize_pcl(cloud_ptr, map_resolution);
      }

    }


    std::vector<Eigen::Vector3d> path_points;
    path_points.push_back(robot_pos);
    path_points.push_back(destination);

    publishPath(path_points);



    auto centroids = RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles, size_neighbors_and_obstacles,
                                        encumbrance, active_wp, beta, neighbors_and_obstacles_noisy);
    auto centroids_no_rotation = RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles, size_neighbors_and_obstacles, encumbrance,
                                    {goal[0], goal[1], goal[2]}, beta, neighbors_and_obstacles_noisy);

    std::vector<double> c1         = {std::get<0>(centroids).x(), std::get<0>(centroids).y(), std::get<0>(centroids).z()};
    std::vector<double> c2         = {std::get<1>(centroids).x(), std::get<1>(centroids).y(), std::get<1>(centroids).z()};
    std::vector<double> c1_no_conn = {std::get<2>(centroids).x(), std::get<2>(centroids).y(), std::get<2>(centroids).z()};

    std::vector<double> c1_no_rotation = {std::get<2>(centroids_no_rotation).x(), std::get<2>(centroids_no_rotation).y(), std::get<2>(centroids_no_rotation).z()};

    std::vector<double> current_position(3);  // Vector with two elements
    current_position[0] = robot_pos[0];    // Assigning first element
    current_position[1] = robot_pos[1];
    current_position[2] = robot_pos[2];

    RBLController::apply_rules(beta, c1_no_conn, c2, current_position, dt, beta_min, betaD, goal, d1, th, d2, d3, d4, d5, d6, d7, destination, c1_no_rotation);

    c1_to_rviz = c1_no_conn;

    if (use_livox_tilted) {
      double desired_heading = std::atan2(c1_no_conn[1] - robot_pos[1], c1_no_conn[0] - robot_pos[0]);
      p_ref.heading = desired_heading;
      double diff = std::fmod(desired_heading - roll_pitch_yaw[2] + M_PI, 2 * M_PI) - M_PI;
      double difference = (diff < -M_PI) ? diff + 2 * M_PI : diff;

      if (std::abs(difference) < M_PI/4) {
        p_ref.position.x = c1_no_conn[0];  // next_values[0];
        p_ref.position.y = c1_no_conn[1];
        p_ref.position.z = c1_no_conn[2];
      } else {
        p_ref.position.x = robot_pos[0];  // next_values[0];
        p_ref.position.y = robot_pos[1];
        p_ref.position.z = robot_pos[2];
      }
    } else {
      p_ref.position.x = c1_no_conn[0];  // next_values[0];
      p_ref.position.y = c1_no_conn[1];
      p_ref.position.z = c1_no_conn[2];
    }

    saveUAVPositionToCSV(_uav_name_, robot_pos, "uav_positions.csv");

  }

  if (save_scene_to_csv) {
    std::cout << "All saved setting save_scene_to_csv as false" << std::endl;
    save_scene_to_csv = false;
  }

  // set drone ref
  mrs_msgs::ReferenceStampedSrv srv;
  srv.request.reference       = p_ref;
  srv.request.header.frame_id = _control_frame_;
  srv.request.header.stamp    = ros::Time::now();

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  if (sc_set_position_.call(srv)) {

  } else {
    ROS_ERROR_THROTTLE(3.0, "Failed to call service ref_pos_out");
  }
  if (!flag_stop && robot_pos[1] > 0) {
    flag_stop                    = true;
    ros::Time     end_time_1     = ros::Time::now();
    ros::Duration elapsed_time_1 = end_time_1 - start_time_1;
    // ROS_INFO("Elapsed time: %.2f seconds", elapsed_time_1.toSec());
  }
}
//}


pcl::PointCloud<pcl::PointXYZ> RBLController::voxelize_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double map_resolution) {
  pcl::PointCloud<pcl::PointXYZ> geometric_center_cloud;

  if (!cloud_ptr || cloud_ptr->empty()) {
    std::cerr << "Warning: Input cloud is null or empty. Returning an empty point cloud." << std::endl;
    return geometric_center_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> centroid_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> sor; 

  sor.setInputCloud(cloud_ptr); 
  sor.setLeafSize(static_cast<float>(map_resolution), static_cast<float>(map_resolution), static_cast<float>(map_resolution));
  
  sor.filter(centroid_cloud);

  geometric_center_cloud.reserve(centroid_cloud.size());

  for (const auto& centroid_point : centroid_cloud.points) {
    pcl::PointXYZ geo_center; // A point to store the calculated geometric center

    geo_center.x = static_cast<float>(std::round(centroid_point.x / map_resolution) * map_resolution);
    geo_center.y = static_cast<float>(std::round(centroid_point.y / map_resolution) * map_resolution);
    geo_center.z = static_cast<float>(std::round(centroid_point.z / map_resolution) * map_resolution);

    Eigen::Vector3d point;
    point.x() = geo_center.x;
    point.y() = geo_center.y;
    point.z() = geo_center.z;

    
    if ( (robot_pos-point).norm() <= radius_sensing) {
      geometric_center_cloud.points.push_back(geo_center); // Add the calculated geometric center
    }
  }

  geometric_center_cloud.width = geometric_center_cloud.points.size();
  geometric_center_cloud.height = 1; // Unordered point cloud

  return geometric_center_cloud;
}




void RBLController::saveUAVPositionToCSV(const std::string& uav_name, const Eigen::Vector3d& position, const std::string& filename) {
  std::ofstream file;
  file.open(filename, std::ios::app); // Open in append mode
  ros::Duration elapsed_time = ros::Time::now() - starting_time;
  ros::spinOnce();
  double vel_norm = velocity.norm();
  //  Get current working directory
  std::string current_path = std::filesystem::current_path().string();
  // std::cout << "Saving to csv now!!! PWD: " << current_path << std::endl;

  if (file.is_open()) {
    file << uav_name << "," << position.x() << "," << position.y() << "," << position.z() << "," << elapsed_time.toSec() << "," << vel_norm << "\n";
    file.close();
  } else {
    std::cerr << "Error: Unable to open file " << filename << std::endl;
  }
}

void RBLController::savePCL2TOCSV(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& filename) {
  std::ofstream file;
  file.open(filename, std::ios::app); // Open in append mode

  // Get current working directory
  std::string current_path = std::filesystem::current_path().string();
  // std::cout << "Saving PCL2 to csv now!!! PWD: " << current_path << std::endl;

  if (file.is_open()) {
    for (const auto& point : cloud->points) {
      file << point.x << "," << point.y << "," << point.z << "\n";
    }
    file.close();
  } else {
    std::cerr << "Error: Unable to open file " << filename << std::endl;
  }
}

void RBLController::velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  // Access the velocity components
  velocity = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z); 
}


Eigen::Vector3d RBLController::compute_reference_point(Eigen::Vector3d robot_pos, std::vector<double> centroid){
  assert(centroid.size() == 3 && "Centroid must have exactly 3 elements!");
  Eigen::Vector3d centroid_vec(centroid[0], centroid[1], centroid[2]);
  Eigen::Vector3d reference_point = robot_pos - centroid_vec;
  return reference_point;
}

/* callbackTimerDiagnostics() //{ */
void RBLController::callbackTimerDiagnostics([[maybe_unused]] const ros::TimerEvent &te) {

  if (!is_initialized_) {
    return;
  }

  bool timeout_exceeded = false;


  try {
    publishDestination();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_destination_.getTopic().c_str());
  }

  try {
    publishPosition();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_position_.getTopic().c_str());
  }

  try {
    publishPcl();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_pointCloud_.getTopic().c_str());
  }

  try {
    publishCentroid();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_centroid_.getTopic().c_str());
  }

}
//}

void RBLController::pointCloud2Callback(const sensor_msgs::PointCloud2& pcl_cloud2) {
  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>); 

  pcl::fromROSMsg(pcl_cloud2, *temp_cloud);

  cloud = *temp_cloud;
  ROS_INFO_STREAM("Received point cloud with " << cloud.size() << "points.");
}



/*ServicesCallbacks //{ */

/* activationServiceCallback() //{ */
bool RBLController::activationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for activation of planning
  ROS_INFO("[RBLController]: Activation service called.");
  res.success = true;

  if (control_allowed_) {
    res.message = "Control was already allowed.";
    ROS_WARN("[RBLController]: %s", res.message.c_str());
  } else {
    control_allowed_ = true;
    starting_time = ros::Time::now();
    res.message      = "Control allowed.";
    ROS_INFO("[RBLController]: %s", res.message.c_str());
  }

  start_time_1 = ros::Time::now();
  ROS_INFO("Start time activation: %.2f seconds", start_time_1.toSec());
  return true;
}
//}

/* deactivationServiceCallback() //{ */
bool RBLController::deactivationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for deactivation of planning
  ROS_INFO("[RBLController]: Deactivation service called.");
  res.success = true;

  if (!control_allowed_) {
    res.message = "Control was already disabled.";
    ROS_WARN("[RBLController]: %s", res.message.c_str());
  } else {
    control_allowed_ = false;
    res.message      = "Control disabled.";
    ROS_INFO("[RBLController]: %s", res.message.c_str());
  }

  return true;
}
//}

/* saveToCsvServiceCallback() //{ */
bool RBLController::saveToCsvServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for deactivation of planning
  ROS_INFO("[RBLController]: SaveToCsv service called.");
  res.success = true;

  if (save_scene_to_csv) {
    res.message = "Already saving!";
    ROS_WARN("[RBLController]: %s", res.message.c_str());
  } else {
    save_scene_to_csv = true;
    res.message      = "Saving relevant pcl2, meshes and so on...";
    ROS_INFO("[RBLController]: %s", res.message.c_str());
  }
  return true;
}
//}


/* flyToStartServiceCallback() //{ */
bool RBLController::flyToStartServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for activation of planning
  ROS_INFO("[RBLController]: Fly to start service called.");
  res.success = true;

  mrs_msgs::Vec4 srv;
  srv.request.goal = {_required_initial_position_[0], _required_initial_position_[1], _required_initial_position_[2], 0.0};
  ;
  if (sc_goto_position_.call(srv)) {
    if (srv.response.success) {
      res.message          = "Fly to start called.";
      fly_to_start_called_ = true;
    } else {
      res.success = false;
      res.message = "GoTo service returned success false.";
    }
  } else {
    res.success = false;
    res.message = "Call to GoTo service failed.";
  }

  ROS_INFO("[RBLController]: %s", res.message.c_str());


  return true;
}
//}
//}

// | -------------------- nodelet macro ----------------------- |
PLUGINLIB_EXPORT_CLASS(formation_control::RBLController, nodelet::Nodelet);
}  // namespace formation_control
