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
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <mrs_msgs/Vec4.h>
#include <sensor_msgs/LaserScan.h>
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


// helper libraries
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <deque>
#include <utility>
#include <chrono>
namespace formation_control
{

class RBLController : public nodelet::Nodelet {

public:
  virtual void onInit();
  // general variables
  bool is_initialized_ = false;

  std::vector<std::pair<double, double>>              new_neighbors;
  std::vector<std::vector<std::pair<double, double>>> neighbors_past_measurements;  // For storing past measurements
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

  // trigger service
  ros::ServiceServer service_activate_control_;
  ros::ServiceServer service_deactivate_control_;
  bool               control_allowed_ = false;
  bool               activationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool               deactivationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

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
  ros::Publisher pub_obstacles_;
  ros::Publisher pub_neighbors_;
  ros::Publisher pub_destination_;
  ros::Publisher pub_position_;
  ros::Publisher pub_centroid_;
  ros::Publisher pub_hull_;
  void           publishObstacles();
  void           publishDestination();
  void           publishPosition();
  void           publishNeighbors();
  void           publishHull();
  void           publishCentroid();

  std::vector<ros::Subscriber> clusters_sub_;
  std::vector<ros::Subscriber> clusters_sub_1;
  void                         clustersCallback(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg);
  void                         clustersCallback1(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg);
  void                         callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &array_pose);
  /* UVDAR */
  std::vector<ros::Subscriber> sub_uvdar_filtered_poses_;

  // | --------------------------- timer callbacks ----------------------------- |

  void                                                callbackTimerPubNeighbors(const ros::TimerEvent &event);
  ros::Timer                                          timer_pub_neighbors_;
  ros::Publisher                                      neigbor_pub_;
  std::map<unsigned int, geometry_msgs::PointStamped> neighbors_position_;
  std::mutex                                          mutex_neighbors_position_;

  // formation control variables
  std::pair<double, double>              robot_pos;
  double                                 step_size;
  double                                 radius;
  std::vector<std::pair<double, double>> neighbors;
  std::vector<std::pair<double, double>> neighbors_and_obstacles;
  std::vector<std::pair<double, double>> neighbors_and_obstacles_noisy;
  std::vector<std::pair<double, double>> all_uavs;
  std::vector<std::pair<double, double>> hull_voro;
  std::pair<double, double>              val = {1000.0, 1000.0};
  std::vector<std::pair<double, double>> fixed_neighbors_vec;
  std::pair<double, double>              destination;
  std::vector<std::pair<double, double>> obstacles;
  std::vector<std::pair<double, double>> obstacles_;
  std::vector<std::pair<double, double>> obstacles1_;
  std::vector<std::pair<double, double>> obstacles_nofiltered;

  std::vector<std::pair<double, double>> tracked_obs;  // std::make_pair(1,(0.0, 0.0});
  std::vector<double>                    probabilities_  = {0};
  std::vector<int>                       missing_counts_ = {0};
  std::vector<bool>                      matched_        = {false};
  std::vector<double>                    goal            = {0, 0};
  std::vector<double>                    goal_original   = {0, 0};
  bool                                   has_this_pose_  = false;
  std::vector<double>                    size_neighbors_and_obstacles;
  std::vector<double>                    size_neighbors;
  std::vector<double>                    size_obstacles;
  std::vector<std::deque<double>>        x_windows;
  std::vector<std::deque<double>>        y_windows;
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
  double                                 th = 0;
  double                                 maximum_distance_conn;
  double                                 noisy_measurements;
  double                                 noisy_angle;
  double                                 threshold;
  double                                 bias_error;
  int                                    window_length;
  double                                 cwvd_rob;
  double                                 cwvd_obs;
  double                                 cwvd;
  double                                 refZ_;
  bool                                   simulation_;
  std::mutex                             mutex_uav_odoms_;
  std::mutex                             mutex_uav_uvdar_;
  std::mutex                             mutex_obstacles_;
  std::string                            _odometry_topic_name_;
  ros::Subscriber                        uav_odom_subscriber_;
  /* void                         odomCallback(const nav_msgs::OdometryConstPtr &msg, int idx); */
  void                         odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  std::vector<Eigen::Vector3d> uav_positions_;
  Eigen::Vector3d              uav_position_;
  std::vector<Eigen::Vector3d> uav_neighbors_;
  std::vector<double>          largest_eigenvalue_;
  std::vector<ros::Time>       last_odom_msg_time_;
  double                       _odom_msg_max_latency_;
  double                       max_obstacle_integration_dist_sqr_;


  void publishObstacles(ros::Publisher &pub, const std::vector<std::pair<double, double>> &obstacles);
  void publishCentroid(ros::Publisher &pub, const std::vector<double> &c1);
  void publishNeighbors(ros::Publisher &pub, const std::vector<std::pair<double, double>> &neighbors_and_obstacles_noisy);
  void publishHull(ros::Publisher &pub, const std::vector<std::pair<double, double>> &hull_voro);


  std::vector<std::pair<double, double>> points_inside_circle(std::pair<double, double> robot_pos, double radius, double step_size);


  std::vector<std::pair<double, double>> fixed_neighbors(const std::vector<std::pair<double, double>> &positions, const std::vector<int> &adjacency_matrix,
                                                         size_t my_index);

  std::vector<std::pair<double, double>> insert_pair_at_index(const std::vector<std::pair<double, double>> &vec, size_t idx,
                                                              const std::pair<double, double> &value);

  std::vector<std::pair<double, double>> communication_constraint(const std::vector<std::pair<double, double>> &points,
                                                                  const std::vector<std::pair<double, double>> &neighbors);

  std::vector<std::pair<double, double>> find_closest_points(const std::pair<double, double> &robot_pos, const std::vector<std::pair<double, double>> &points,
                                                             const std::vector<std::pair<double, double>> &neighbors, const std::vector<double> &only_robots);

  std::vector<double> compute_scalar_value(const std::vector<double> &x_test, const std::vector<double> &y_test, const std::pair<double, double> &destination,
                                           double beta);

  std::vector<std::pair<double, double>> account_encumbrance(const std::vector<std::pair<double, double>> &points, const std::pair<double, double> &robot_pos,
                                                             const std::vector<std::pair<double, double>> &neighbors, const std::vector<double> &size_neighbors,
                                                             double encumbrance);

  std::tuple<std::pair<double, double>, std::pair<double, double>, std::pair<double, double>> get_centroid(
      std::pair<double, double> robot_pos, double radius, double step_size, std::vector<std::pair<double, double>> &neighbors,
      const std::vector<double> &size_neighbors, std::vector<std::pair<double, double>> &neighbors_and_obstacles,
      const std::vector<double> &size_neighbors_and_obstacles, double encumbrance, const std::pair<double, double> &destination, double &beta,
      std::vector<std::deque<double>> &x_windows, std::vector<std::deque<double>> &y_windows,
      std::vector<std::pair<double, double>> &neighbors_and_obstacles_noisy);


  typedef std::pair<double, double> Point;

  double                           euclideanDistance(const Point &a, const Point &b);
  double                           cross(const Point &O, const Point &A, const Point &B);
  bool                             isInsideConvexPolygon(const std::vector<std::pair<double, double>> &polygon, const std::pair<double, double> &testPoint);
  std::vector<Point>               convexHull(std::vector<Point> points);
  void apply_rules(double &beta, const std::vector<double> &c1, const std::vector<double> &c2, const std::vector<double> &current_position, double dt,
                   double beta_min, const double &betaD, std::vector<double> &goal, double d1, double &th, double d2, double d3, double d4,
                   std::pair<double, double> &destinations, std::vector<double> &c1_no_rotation);


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
