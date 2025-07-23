#ifndef RBL_CONTROLLER_H
#define RBL_CONTROLLER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// I/O
#include <stdio.h>

// matrix math
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

// messages
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
#include <sensor_msgs/PointCloud2.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

// custom helper functions from mrs library
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/Float64Stamped.h>



// #include <process_pointcloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <filesystem>
#include <omp.h>

// helper libraries
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
#include "rbl_controller/ActivateParams.h"

namespace formation_control
{

class RBLController : public nodelet::Nodelet {

public:
  virtual void onInit();
  // general variables
  bool                                                is_initialized_ = false;
  bool                                                flag_replanner  = true;
  std::vector<Eigen::Vector3d>                        new_neighbors;
  std::vector<std::vector<Eigen::Vector3d>>           neighbors_past_measurements;  // For storing past measurements
  // parameters from config file definitions
  int                      n_drones_;  // Number of drones
  std::vector<std::string> _uav_names_;
  std::map<int, int>       _uav_uvdar_ids_;
  std::string              _uav_name_;
  std::string              _target_uav_name_;
  std::string              _control_frame_;
  int                      this_uav_idx_;
  double                   _target_gain_;
  int                      _c_dimensions_;  // controlled dimensions
  ros::Time                start_time_1;
  bool                     flag_stop = false;
  ros::ServiceClient       sc_set_velocity_;
  ros::ServiceClient       sc_set_position_;
  ros::Timer               timer_set_reference_;
  ros::Timer               timer_set_active_wp_;
  ros::Timer               timer_pub_;
  int                      _rate_timer_set_reference_;
  void                     callbackTimerSetReference([[maybe_unused]] const ros::TimerEvent &te);
  // void callbackPublisher([[maybe_unused]] const ros::TimerEvent &te);

  // diag timer
  ros::Timer timer_diagnostics_;
  int        _rate_timer_diagnostics_;
  bool       all_robots_positions_valid_ = true;
  void       callbackTimerDiagnostics([[maybe_unused]] const ros::TimerEvent &te);
  double     _odom_timeout_;

  void goalUpdateLoop(const ros::TimerEvent&);
  // trigger service
  ros::ServiceServer service_activate_control_;
  ros::ServiceServer service_activate_params_control_;
  ros::ServiceServer service_deactivate_control_;
  ros::ServiceServer service_save_to_csv_;
  bool               control_allowed_ = false;
  bool               activationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool               deactivationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool               activationParamsServiceCallback(rbl_controller::ActivateParams::Request &req, rbl_controller::ActivateParams::Response &res);

  // trigger goto service
  ros::ServiceServer service_fly_to_start_;
  bool               fly_to_start_called_ = false;
  bool               flyToStartServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  ros::ServiceClient sc_goto_position_;
  Eigen::Vector3d    _required_initial_position_;
  bool               is_at_initial_position_ = true;
  double             _dist_to_start_limit_;
  double             getDistToInitialPosition();

  Eigen::Vector3d _monitored_area_origin_;
  // visualization publishing
  ros::Publisher pub_destination_;
  ros::Publisher pub_position_;
  ros::Publisher pub_centroid_;
  void           publishDestination();
  void           publishPosition();
  void           publishCentroid();

  std::vector<ros::Subscriber> clusters_sub_;
  std::vector<ros::Subscriber> clusters_sub_1;
  std::vector<ros::Subscriber> waypoints_sub_;
  ros::Subscriber waypoint_sub;
  void                         clustersCallback(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg);
  void                         clustersCallback1(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg);
  // void                         waypointsCallback(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg);
  void                         waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

  void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);

  void                         callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &array_pose);
  /* UVDAR */
  std::vector<ros::Subscriber> sub_uvdar_filtered_poses_;
  std::vector<ros::Subscriber> other_uav_odom_subscribers_;

  // | --------------------------- timer callbacks ----------------------------- |

  void                                                callbackTimerPubNeighbors(const ros::TimerEvent &event);
  ros::Timer                                          timer_pub_neighbors_;
  ros::Publisher                                      neigbor_pub_;
  std::map<unsigned int, geometry_msgs::PointStamped> neighbors_position_;
  std::mutex                                          mutex_neighbors_position_;

  // formation control variables
  Eigen::Vector3d                        robot_pos;
  double                                 step_size;
  double                                 radius;
  std::vector<Eigen::Vector3d>           neighbors;
  std::vector<Eigen::Vector3d>           neighbors_and_obstacles;
  std::vector<Eigen::Vector3d>           neighbors_and_obstacles_noisy;
  std::vector<Eigen::Vector3d>           all_uavs;
  std::vector<Eigen::Vector3d>           hull_voro;
  std::pair<double, double>              val = {1000.0, 1000.0};
  std::vector<Eigen::Vector3d>           fixed_neighbors_vec;
  Eigen::Vector3d                        destination;
  Eigen::Vector3d                        active_wp;
  std::vector<Eigen::Vector3d>           obstacles;
  std::vector<Eigen::Vector3d>           obstacles_;
  std::vector<std::pair<double, double>> waypoints_;
  std::vector<Eigen::Vector3d>           obstacles1_;
  std::vector<Eigen::Vector3d>           obstacles_nofiltered;

  std::vector<std::pair<double, double>> tracked_obs;  // std::make_pair(1,(0.0, 0.0});
  std::vector<double>                    probabilities_  = {0};
  std::vector<int>                       missing_counts_ = {0};
  std::vector<bool>                      matched_        = {false};
  std::vector<double>                    goal            = {0, 0, 0};
  std::vector<double>                    goal_original   = {0, 0, 0};
  bool                                   has_this_pose_  = false;
  std::vector<double>                    size_neighbors_and_obstacles;
  std::vector<double>                    size_neighbors;
  std::vector<double>                    size_obstacles;
  double                                 size_neighbors1;
  double                                 size_obstacles1;
  double                                 encumbrance;
  double                                 dt;
  double                                 beta_min;
  double                                 betaD;
  double                                 beta;
  int                                    init_flag;
  std::vector<int>                       Adj_matrix;
  double                                 d1;
  double                                 d2;
  double                                 d3;
  double                                 d4;
  double                                 d5;
  double                                 d6;
  double                                 d7;
  double                                 th = 0;
  double                                 ph = 0;
  double                                 maximum_distance_conn;
  double                                 noisy_measurements;
  double                                 noisy_angle;
  double                                 threshold;
  double                                 bias_error;
  int                                    window_length;
  double                                 cwvd_rob;
  double                                 cwvd_obs;
  double                                 cwvd;
  double                                 searchRadius;
  double                                 radius_sensing;
  double                                 refZ_;
  double                                 min_z;
  double                                 max_z;
  bool                                   flag_3D;
  bool                                   use_z_rule;
  bool                                   simulation_;
  bool                                   connectivity_flag;
  std::mutex                             mutex_uav_odoms_;
  std::mutex                             mutex_uav_uvdar_;
  std::mutex                             mutex_obstacles_;
  std::mutex                             mutex_waypoints_;
  std::mutex                             points_mutex_;  
  std::vector<geometry_msgs::Point>      dense_points_;
  std::string                            _odometry_topic_name_;
  ros::Subscriber                        uav_odom_subscriber_;
  void                         odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void                         odomCallback(const nav_msgs::OdometryConstPtr &msg, int idx); 
  std::vector<Eigen::Vector3d> uav_positions_;
  Eigen::Vector3d              uav_position_;
  Eigen::Vector3d              roll_pitch_yaw;
  std::vector<Eigen::Vector3d> uav_neighbors_;
  std::vector<double>          largest_eigenvalue_;
  std::vector<ros::Time>       last_odom_msg_time_;
  double                       _odom_msg_max_latency_;
  double                       max_obstacle_integration_dist_sqr_;
  bool                         use_livox_tilted;
  double                       livox_tilt_deg;
  double                       livox_fov;
  Eigen::Vector3d              livox_translation;

  ros::Subscriber                 sub_pointCloud2_;
  ros::Subscriber                 sub_neighbors_;
  pcl::PointCloud<pcl::PointXYZ>  cloud;
  pcl::PointCloud<pcl::PointXYZ>  processed_cloud;
  pcl::PointCloud<pcl::PointXYZ>  downsampled_cloud;
  void neighborCallback(const sensor_msgs::PointCloud2& neighbors_pcl);
  void pointCloud2Callback(const sensor_msgs::PointCloud2& pcl_cloud2);
  ros::Publisher                 pub_pointCloud_; //for rviz
  void publishPcl();
  ros::Publisher                 pub_cellA_;
  void publishCellA(std::vector<Eigen::Vector3d> points);
  ros::Publisher                 pub_cell_sensed_A_;
  void publishCellActivelySensedA(std::vector<Eigen::Vector3d> points);
  ros::Publisher                 pub_planes_;
  void publishPlanes(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& planes);
  ros::Publisher                 pub_norms_;
  void publishNorms(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& planes);
  ros::Publisher                 pub_path_;
  void publishPath(const std::vector<Eigen::Vector3d>& path);
  size_t                         last_marker_count = 0;
  std::vector<double>            c1_to_rviz = {0, 0, 0};
  bool                           use_bonxai_mapping;
  // bool                           use_voxel;
  double                         map_resolution;
  Eigen::Vector3d closest_point_from_voxel(Eigen::Vector3d robot_pos, Eigen::Vector3d voxel_center, double map_resolution);
  std::vector<Eigen::Vector3d> find_closest_points_using_voxel(const Eigen::Vector3d                        &robot_pos,
                                                              const std::vector<Eigen::Vector3d>           &points,
                                                              const std::vector<Eigen::Vector3d>           &neighbors,
                                                              pcl::PointCloud<pcl::PointXYZ>  cloud);
  std::vector<Eigen::Vector3d> find_closest_points_using_voxel_fast(const Eigen::Vector3d                        &robot_pos,
                                                              const std::vector<Eigen::Vector3d>           &points,
                                                              const std::vector<Eigen::Vector3d>            &boundary_cell_A_points,
                                                              const std::vector<Eigen::Vector3d>           &neighbors,
                                                              pcl::PointCloud<pcl::PointXYZ>  cloud);
  std::vector<Eigen::Vector3d> find_closest_points_using_voxel_faster(const Eigen::Vector3d                        &robot_pos,
                                                              const std::vector<Eigen::Vector3d>           &points,
                                                              const std::vector<Eigen::Vector3d>            &boundary_cell_A_points,
                                                              const std::vector<Eigen::Vector3d>           &neighbors,
                                                              pcl::PointCloud<pcl::PointXYZ>  cloud);

  Eigen::Matrix3d Rx(double angle);
  Eigen::Matrix3d Ry(double angle);
  Eigen::Matrix3d Rz(double angle);

  pcl::PointCloud<pcl::PointXYZ> voxelize_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double map_resolution);
  
  //TODO del
  bool                         save_scene_to_csv;
  void saveUAVPositionToCSV(const std::string& uav_name, const Eigen::Vector3d& position, const std::string& filename);
  bool saveToCsvServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void savePCL2TOCSV(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& filename);
  void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  ros::Time                    starting_time;
  ros::Subscriber              sub_velocity_;
  Eigen::Vector3d              velocity;

  pcl::PolygonMesh             mesh;
  double point_triangle_distance(const Eigen::Vector3f& point, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3);
  std::vector<Eigen::Vector3d> pointCloudToEigenVector(const pcl::PointCloud<pcl::PointXYZ>& cloud); 
  Eigen::Vector3d closest_point_from_triangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
  std::vector<Eigen::Vector3d> slice_sphere(std::vector<Eigen::Vector3d> points, Eigen::Vector3d robot_pos, double angle_deg, double livox_fov, Eigen::Vector3d roll_pitch_yaw);
  Eigen::Vector3d closest_in_legal_set(Eigen::Vector3d c_pos, std::vector<Eigen::Vector3d> legal_set, Eigen::Vector3d robot_pos, double step_size);

  void publishObstacles(ros::Publisher &pub, const std::vector<std::pair<double, double>> &obstacles);
  void publishCentroid(ros::Publisher &pub, const std::vector<double> &c1);
  void publishNeighbors(ros::Publisher &pub, const std::vector<Eigen::Vector3d> &neighbors_and_obstacles_noisy);
  void publishHull(ros::Publisher &pub, const std::vector<Eigen::Vector3d> &hull_voro);


  std::vector<Eigen::Vector3d> points_inside_circle(Eigen::Vector3d robot_pos, double radius, double step_size);
  std::vector<Eigen::Vector3d> points_inside_sphere(Eigen::Vector3d robot_pos, double radius, double step_size);
  std::vector<Eigen::Vector3d> boundary_points_sphere(Eigen::Vector3d robot_pos, double radius, double step_size);
  std::vector<Eigen::Vector3d> project_boundary_points_on_encumbrance(Eigen::Vector3d robot_pos, double encumbrance, std::vector<Eigen::Vector3d> boundary_points);


  std::vector<Eigen::Vector3d> fixed_neighbors(const std::vector<Eigen::Vector3d> &positions, const std::vector<int> &adjacency_matrix,
                                                         size_t my_index);

  std::vector<Eigen::Vector3d> insert_vec3d_at_index(const std::vector<Eigen::Vector3d> &vec, size_t idx,
                                                              const Eigen::Vector3d &value);

  std::vector<Eigen::Vector3d> communication_constraint(const std::vector<Eigen::Vector3d> &points,
                                                                  const std::vector<Eigen::Vector3d> &neighbors);

  std::vector<Eigen::Vector3d> find_closest_points(const Eigen::Vector3d &robot_pos, const std::vector<Eigen::Vector3d> &points,
                                                             const std::vector<Eigen::Vector3d> &neighbors, const std::vector<double> &only_robots, const pcl::PolygonMesh &mesh);

  std::vector<double> compute_scalar_value(const std::vector<double> &x_test, const std::vector<double> &y_test, const std::vector<double> &z_test, const Eigen::Vector3d &destination,
                                           double beta);

  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> get_centroid(
      Eigen::Vector3d robot_pos, double radius, double step_size, std::vector<Eigen::Vector3d> &neighbors,
      const std::vector<double> &size_neighbors, std::vector<Eigen::Vector3d> &neighbors_and_obstacles,
      const std::vector<double> &size_neighbors_and_obstacles, double encumbrance, const Eigen::Vector3d &destination, double &beta,
      std::vector<Eigen::Vector3d> &neighbors_and_obstacles_noisy);

  Eigen::Vector3d compute_reference_point(Eigen::Vector3d robot_pos, std::vector<double> centroid);

  void apply_rules(double &beta, const std::vector<double> &c1, const std::vector<double> &c2, const std::vector<double> &current_position, double dt,
                   double beta_min, const double &betaD, std::vector<double> &goal, double d1, double &th, double d2, double d3, double d4, double d5, double d6, double d7,
                   Eigen::Vector3d &destination, std::vector<double> &c1_no_rotation);


  std::mutex                                          mutex_position_command_;
  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> sh_position_command_;
  void                                                getPositionCmd();
  geometry_msgs::Point                                position_command_;  // position of controlled UAV
  bool                                                got_position_command_ = false;

  // transformer
  std::shared_ptr<mrs_lib::Transformer> transformer_;
};


}  // namespace formation_control
#endif
