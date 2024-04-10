// cored
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// I/O
#include <stdio.h>

// matrix math
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

// messages
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrackerCommand.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

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

namespace formation_control
{

  // | -------------------- class definition --------------------- |
  class RBLController : public nodelet::Nodelet
  {
  public:
    virtual void onInit();

  private:
    // general variables
    bool is_initialized_ = false;

    // parameters from config file definitions
    int n_drones_; // Number of drones
    std::vector<std::string> _uav_names_;
    std::string _uav_name_;
    std::string _target_uav_name_;
    std::string _control_frame_;
    int this_uav_idx_;
    double _target_gain_;
    int _c_dimensions_; // controlled dimensions
    // set reference timer
    ros::ServiceClient sc_set_velocity_;
    ros::ServiceClient sc_set_position_;
    ros::Timer timer_set_reference_;
    int _rate_timer_set_reference_;
    void callbackTimerSetReference([[maybe_unused]] const ros::TimerEvent &te);

    // diag timer
    ros::Timer timer_diagnostics_;
    int _rate_timer_diagnostics_;
    bool all_robots_positions_valid_ = false;
    void callbackTimerDiagnostics([[maybe_unused]] const ros::TimerEvent &te);
    double _odom_timeout_;

    // trigger service
    ros::ServiceServer service_activate_control_;
    bool control_allowed_ = false;
    bool activationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    // formation control variables
    std::pair<double, double> robot_pos;
    double step_size;
    double radius;
    std::vector<std::pair<double, double>> neighbors;
    std::vector<std::pair<double, double>> all_uavs;
    std::pair<double, double> val = {1000.0, 1000.0};

    std::vector<std::pair<double, double>> fixed_neighbors_vec;

    std::pair<double, double> destination;
    // std::vector<double> destinations;
    std::vector<double> goal = {0, 0};
    std::vector<double> size_neighbors = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    double encumbrance;
    double dt;
    double beta_min;
    double betaD;
    double beta;
    std::vector<std::vector<int>> Adj_matrix = {{0, 1, 1, 0, 0, 0, 0},
                                                {1, 0, 1, 1, 0, 0, 0},
                                                {1, 1, 0, 1, 1, 0, 0},
                                                {0, 1, 1, 0, 1, 1, 0},
                                                {0, 0, 1, 1, 0, 1, 1},
                                                {0, 0, 0, 1, 1, 0, 1},
                                                {0, 0, 0, 0, 1, 1, 0}};
    /*std::vector<std::vector<int>> Adj_matrix = {
        {0, 1, 1},
        {1, 0, 1},
        {1, 1, 0},
    };*/

    double d1;
    double d2;
    double d3;
    double d4;
    double th = 0;

    // callbacks definitions
    std::mutex mutex_uav_odoms_;
    std::string _odometry_topic_name_;
    std::vector<ros::Subscriber> other_uav_odom_subscribers_;
    void odomCallback(const nav_msgs::OdometryConstPtr &msg, int idx);
    std::vector<Eigen::Vector3d> uav_positions_;

    std::vector<ros::Time> last_odom_msg_time_;
    double _odom_msg_max_latency_;

    std::vector<std::pair<double, double>> points_inside_circle(std::pair<double, double> robot_pos, double radius, double step_size);

    std::vector<std::pair<double, double>> fixed_neighbors(const std::vector<std::pair<double, double>> &positions, const std::vector<std::vector<int>> &adjacency_matrix, size_t my_index);

    std::vector<std::pair<double, double>> insert_pair_at_index(const std::vector<std::pair<double, double>> &vec, size_t idx, const std::pair<double, double> &value);

    std::vector<std::pair<double, double>> communication_constraint(
        const std::vector<std::pair<double, double>> &points,
        const std::vector<std::pair<double, double>> &neighbors);

    std::vector<std::pair<double, double>> find_closest_points(const std::pair<double, double> &robot_pos, const std::vector<std::pair<double, double>> &points, const std::vector<std::pair<double, double>> &neighbors);

    std::vector<double> compute_scalar_value(const std::vector<double> &x_test, const std::vector<double> &y_test, const std::pair<double, double> &destination, double beta);

    std::vector<std::pair<double, double>> account_encumbrance(const std::vector<std::pair<double, double>> &points, const std::pair<double, double> &robot_pos, const std::vector<std::pair<double, double>> &neighbors, const std::vector<double> &size_neighbors, double encumbrance);

    std::tuple<std::pair<double, double>, std::pair<double, double>, std::pair<double, double>>  get_centroid(
        std::pair<double, double> robot_pos,
        double radius,
        double step_size,
        std::vector<std::pair<double, double>> &neighbors,
        const std::vector<double> &size_neighbors,
        double encumbrance,
        const std::pair<double, double> &destination,
        double beta);

    void apply_rules(double &beta,
                     const std::vector<double> &c1,
                     const std::vector<double> &c2,
                     const std::vector<double> &current_position,
                     double dt,
                     double beta_min,
                     const double &betaD,
                     std::vector<double> &goal,
                     double d1,
                     double &th,
                     double d2,
                     double d3,
                     double d4,
                     std::pair<double, double> &destinations,
                     std::vector<double> &c1_no_rotation);

    // reference velocity - UNUSED, left here only as an example
    /* std::mutex                                            mutex_ref_velocity_; */
    /* Eigen::Vector3d                                       ref_velocity_; */
    /* mrs_lib::SubscribeHandler<mrs_msgs::ReferenceStamped> sh_reference_velocity_; */
    /* bool                                                  is_leader_ = false; */
    /* bool                                                  ref_velocity_subscribed_; */
    /* mrs_lib::PublisherHandler<mrs_msgs::ReferenceStamped> ph_reference_velocity_; */
    /* void                                                  publishRefVelocity(); */
    /* void                                                  updateLeadersRefVelocity(); */

    // subscribe position of controlled UAV
    std::mutex mutex_position_command_;
    mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> sh_position_command_;
    void getPositionCmd();
    geometry_msgs::Point position_command_; // position of controlled UAV
    bool got_position_command_ = false;

    // transformer
    std::shared_ptr<mrs_lib::Transformer> transformer_;
  };

  /* onInit() //{ */

  void RBLController::onInit()
  {
    // initialize nodelet
    ros::NodeHandle &nh = getPrivateNodeHandle();
    NODELET_DEBUG("Initializing nodelet...");
    ros::Time::waitForValid();

    // load parameters
    std::string _leader_name;
    mrs_lib::ParamLoader param_loader(nh, "RBLController");
    param_loader.loadParam("uav_names", _uav_names_);
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
    param_loader.loadParam("radius", radius);
    param_loader.loadParam("encumbrance", encumbrance);
    param_loader.loadParam("step_size", step_size);
    param_loader.loadParam("betaD", betaD);
    param_loader.loadParam("beta_min", beta_min);
    param_loader.loadParam("dt", dt);

    // erase this uav from the list of uavs
    auto it = std::find(_uav_names_.begin(), _uav_names_.end(), _uav_name_);

    if (it != _uav_names_.end())
    {
      this_uav_idx_ = it - _uav_names_.begin();
      _uav_names_.erase(it);
    }
    else
    {
      ROS_ERROR("[RBLController]: This UAV is not part of the formation! Check the config file. Shutting down node.");
      ros::shutdown();
    }
    beta = betaD;
    n_drones_ = _uav_names_.size();
    //  = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

    // size_neighbors[n_drones_, encumbrance]; //, encumbrance, encumbrance, encumbrance};
    uav_positions_.resize(n_drones_);
    destination = {0, 18}; //{-10 * cos(2 * this_uav_idx_ * M_PI / (n_drones_ + 1)), -10 * sin(2 * this_uav_idx_ * M_PI / (n_drones_ + 1))};

    goal[0] = destination.first;
    goal[1] = destination.second;

    // std::vector<Eigen::Vector3d> uav_positionsN_(uav_positions_);

    last_odom_msg_time_.resize(n_drones_);

    /* create multiple subscribers to read uav odometries */
    // iterate through drones except this drone and target
    for (int i = 0; i < _uav_names_.size(); i++)
    {

      std::string topic_name = std::string("/") + _uav_names_[i] + std::string("/") + _odometry_topic_name_;
      other_uav_odom_subscribers_.push_back(
          nh.subscribe<nav_msgs::Odometry>(topic_name.c_str(), 1, boost::bind(&RBLController::odomCallback, this, _1, i)));

      ROS_INFO("Subscribing to %s", topic_name.c_str());
    }

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh;
    shopts.node_name = "RBLController";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe = true;
    shopts.autostart = true;
    shopts.queue_size = 10;
    shopts.transport_hints = ros::TransportHints().tcpNoDelay();

    sh_position_command_ = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");

    /* if (is_leader_) { */
    /*   ph_reference_velocity_ = mrs_lib::PublisherHandler<mrs_msgs::ReferenceStamped>(nh, "vel_leader_out", 1); */
    /* } else { */
    /*   sh_reference_velocity_ = mrs_lib::SubscribeHandler<mrs_msgs::ReferenceStamped>(shopts, "vel_leader_in"); */
    /* } */

    // initialize timers
    timer_set_reference_ = nh.createTimer(ros::Rate(_rate_timer_set_reference_), &RBLController::callbackTimerSetReference, this);
    timer_diagnostics_ = nh.createTimer(ros::Rate(_rate_timer_diagnostics_), &RBLController::callbackTimerDiagnostics, this);

    // initialize service servers
    service_activate_control_ = nh.advertiseService("control_activation_in", &RBLController::activationServiceCallback, this);

    // initialize service clients
    sc_set_position_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("ref_pos_out");

    // initialize transformer
    transformer_ = std::make_shared<mrs_lib::Transformer>(nh, "RBLController");
    /* transformer_->setDefaultPrefix(_uav_name_); */
    transformer_->retryLookupNewest(true);

    is_initialized_ = true;
    ROS_INFO("[RBLController]: Initialization completed.");
  }
  //}

  std::vector<std::pair<double, double>> RBLController::points_inside_circle(std::pair<double, double> robot_pos, double radius, double step_size)
  {
    double x_center = robot_pos.first;
    double y_center = robot_pos.second;
    int x_min = static_cast<int>((x_center - radius) / step_size);
    int x_max = static_cast<int>((x_center + radius) / step_size);
    int y_min = static_cast<int>((y_center - radius) / step_size);
    int y_max = static_cast<int>((y_center + radius) / step_size);

    std::vector<double> x_coords, y_coords;
    for (int i = x_min; i <= x_max; ++i)
      x_coords.push_back(i * step_size);
    for (int j = y_min; j <= y_max; ++j)
      y_coords.push_back(j * step_size);

    std::vector<std::pair<double, double>> points;
    for (auto x : x_coords)
    {
      for (auto y : y_coords)
      {
        double distance = std::sqrt(std::pow((x - x_center), 2) + std::pow((y - y_center), 2));
        if (distance <= radius)
          points.push_back(std::make_pair(x, y));
      }
    }
    return points;
  }
  std::vector<std::pair<double, double>> RBLController::insert_pair_at_index(const std::vector<std::pair<double, double>> &vec, size_t idx, const std::pair<double, double> &value)
  {

    std::vector<std::pair<double, double>> new_vec = vec;

    if (idx >= new_vec.size())
    {
      // If the index is out of bounds, push the value to the back of the vector
      new_vec.push_back(value);
    }
    else
    {
      // Insert the value at the specified index
      new_vec.insert(new_vec.begin() + idx, value);
    }

    return new_vec;
  }

  std::vector<std::pair<double, double>> RBLController::fixed_neighbors(const std::vector<std::pair<double, double>> &positions, const std::vector<std::vector<int>> &adjacency_matrix, size_t my_index)
  {
    std::vector<std::pair<double, double>> neighbors_filtered;

    // Iterate over the row of the adjacency matrix corresponding to my_index
    for (size_t j = 0; j < adjacency_matrix[my_index].size(); ++j)
    {
      // Check if the value is 1, indicating a neighbor
      if (adjacency_matrix[my_index][j] == 1)
      {
        // Add the position of the neighbor to neighbors_filtered
        neighbors_filtered.push_back(positions[j]);
      }
    }
    return neighbors_filtered;
  }
  std::vector<std::pair<double, double>> RBLController::communication_constraint(
      const std::vector<std::pair<double, double>> &points,
      const std::vector<std::pair<double, double>> &neighbors)
  {
    std::vector<int> index;
    std::vector<std::pair<double, double>> new_points = points;

    for (const auto &neighbor : neighbors)
    {
      for (size_t i = 0; i < new_points.size(); ++i)
      {
        double distance = std::sqrt(
            std::pow(new_points[i].first - neighbor.first, 2) +
            std::pow(new_points[i].second - neighbor.second, 2));

        if (distance > 4.0)
        {
          index.push_back(i);
        }
      }
    }
    std::vector<std::pair<double, double>>
        filtered_points;
    for (size_t i = 0; i < new_points.size(); ++i)
    {
      if (std::find(index.begin(), index.end(), i) == index.end())
      {
        filtered_points.push_back(new_points[i]);
      }
    }
    return filtered_points;
  }

  std::vector<std::pair<double, double>> RBLController::find_closest_points(const std::pair<double, double> &robot_pos, const std::vector<std::pair<double, double>> &points, const std::vector<std::pair<double, double>> &neighbors)
  {
    std::vector<double> distances_to_robot;
    for (const auto &point : points)
    {
      double distance = std::sqrt(std::pow((point.first - robot_pos.first), 2) + std::pow((point.second - robot_pos.second), 2));
      distances_to_robot.push_back(distance);
    }

    std::vector<std::vector<double>> distances_to_neighbors;
    for (const auto &point : points)
    {
      std::vector<double> distances;
      for (const auto &neighbor : neighbors)
      {
        double distance = std::sqrt(std::pow((point.first - neighbor.first), 2) + std::pow((point.second - neighbor.second), 2));
        distances.push_back(distance);
      }
      distances_to_neighbors.push_back(distances);
    }

    std::vector<std::pair<double, double>> closer_points;
    for (size_t i = 0; i < points.size(); ++i)
    {
      bool is_closer = true;
      for (size_t j = 0; j < neighbors.size(); ++j)
      {
        if (distances_to_robot[i] >= distances_to_neighbors[i][j])
        {
          is_closer = false;
          break;
        }
      }
      if (is_closer)
        closer_points.push_back(points[i]);
    }

    return closer_points;
  }

  std::vector<double> RBLController::compute_scalar_value(const std::vector<double> &x_test, const std::vector<double> &y_test, const std::pair<double, double> &destination, double beta)
  {
    std::vector<double> scalar_values;
    for (size_t i = 0; i < x_test.size(); ++i)
    {
      double distance = std::sqrt(std::pow((x_test[i] - destination.first), 2) + std::pow((y_test[i] - destination.second), 2));
      double scalar_value = std::exp(-distance / beta);
      scalar_values.push_back(scalar_value);
    }
    return scalar_values;
  }

  std::vector<std::pair<double, double>> RBLController::account_encumbrance(const std::vector<std::pair<double, double>> &points, const std::pair<double, double> &robot_pos, const std::vector<std::pair<double, double>> &neighbors, const std::vector<double> &size_neighbors, double encumbrance)
  {
    std::vector<size_t> index;
    double robot_x = robot_pos.first;
    double robot_y = robot_pos.second;

    for (size_t j = 0; j < neighbors.size(); ++j)
    {
      double delta_x = robot_x - neighbors[j].first;
      double delta_y = robot_y - neighbors[j].second;

      if (std::abs(delta_y) < 0.001)
      {
        delta_y = 0.001;
      }
      if (std::abs(delta_x) < 0.001)
      {
        delta_x = 0.001;
      }

      double m = delta_y / delta_x;
      if (std::abs(m) < 0.001)
      {
        m = 0.001;
      }

      double xm = 0.5 * (robot_x + neighbors[j].first);
      double ym = 0.5 * (robot_y + neighbors[j].second);
      double dm = std::sqrt(std::pow(xm - robot_x, 2) + std::pow(ym - robot_y, 2));

      if (dm < size_neighbors[j] + encumbrance)
      {
        std::vector<double> uvec = {delta_x, delta_y};
        double norm_uvec = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
        uvec[0] /= norm_uvec;
        uvec[1] /= norm_uvec;

        double solx = xm + (size_neighbors[j] + encumbrance - dm) * uvec[0];
        double soly = ym + (size_neighbors[j] + encumbrance - dm) * uvec[1];

        if (robot_y + (1 / m) * (robot_x - solx) - soly > 0)
        {
          for (size_t i = 0; i < points.size(); ++i)
          {
            if (points[i].second + (1 / m) * (points[i].first - solx) - soly < 0)
            {
              index.push_back(i);
            }
          }
        }
        else
        {
          for (size_t i = 0; i < points.size(); ++i)
          {
            if (points[i].second + (1 / m) * (points[i].first - solx) - soly > 0)
            {
              index.push_back(i);
            }
          }
        }
      }
    }

    std::vector<std::pair<double, double>> new_points;
    for (size_t i = 0; i < points.size(); ++i)
    {
      if (std::find(index.begin(), index.end(), i) == index.end())
      {
        new_points.push_back(points[i]);
      }
    }
    return new_points;
  }

  std::tuple<std::pair<double, double>, std::pair<double, double>, std::pair<double, double>> RBLController::get_centroid(
      std::pair<double, double> robot_pos,
      double radius,
      double step_size,
      std::vector<std::pair<double, double>> &neighbors,
      const std::vector<double> &size_neighbors,
      double encumbrance,
      const std::pair<double, double> &destination,
      double beta)
  {
    // Get points inside the circle
    std::vector<std::pair<double, double>> circle_points = points_inside_circle(robot_pos, radius, step_size);

    std::vector<std::pair<double, double>> voronoi_circle_intersection;
    std::vector<std::pair<double, double>> voronoi_circle_intersection_connectivity;
    if (!neighbors.empty())
    {
      // Compute the Voronoi cell
      voronoi_circle_intersection = find_closest_points(robot_pos, circle_points, neighbors);
      // Account encumbrance
      voronoi_circle_intersection = account_encumbrance(voronoi_circle_intersection, robot_pos, neighbors, size_neighbors, encumbrance);

      all_uavs = insert_pair_at_index(neighbors, this_uav_idx_, val);

      fixed_neighbors_vec = fixed_neighbors(all_uavs, Adj_matrix, this_uav_idx_);
      //std::cout << "Filtered Neighbors:\n";
      //for (const auto &neighbor : fixed_neighbors_vec)
      //{
      //  std::cout << "(" << neighbor.first << ", " << neighbor.second << ")\n";
      //}
      voronoi_circle_intersection_connectivity = communication_constraint(voronoi_circle_intersection, fixed_neighbors_vec);

      if (voronoi_circle_intersection_connectivity.empty())
      {
        voronoi_circle_intersection_connectivity.push_back(robot_pos);
      }
    }
    else
    {
      voronoi_circle_intersection_connectivity = circle_points;
    }

    std::vector<double> x_in, y_in;
    for (const auto &point : voronoi_circle_intersection_connectivity)
    {
      x_in.push_back(point.first);
      y_in.push_back(point.second);
    }

    std::vector<double> x_in_no_neigh, y_in_no_neigh;
    for (const auto &point : circle_points)
    {
      x_in_no_neigh.push_back(point.first);
      y_in_no_neigh.push_back(point.second);
    }

    std::vector<double> x_in_no_conn, y_in_no_conn;
    for (const auto &point : voronoi_circle_intersection)
    {
      x_in_no_conn.push_back(point.first);
      y_in_no_conn.push_back(point.second);
    }

    // Compute scalar values
    std::vector<double> scalar_values = compute_scalar_value(x_in, y_in, destination, beta);
    std::vector<double> scalar_values_no_neigh = compute_scalar_value(x_in_no_neigh, y_in_no_neigh, destination, beta);
    std::vector<double> scalar_values_no_conn = compute_scalar_value(x_in_no_conn, y_in_no_conn, destination, beta);

    // Compute the weighted centroid
    double sum_x_in_times_scalar_values = 0.0;
    double sum_y_in_times_scalar_values = 0.0;
    double sum_scalar_values = 0.0;

    for (size_t i = 0; i < x_in.size(); ++i)
    {
      sum_x_in_times_scalar_values += x_in[i] * scalar_values[i];
      sum_y_in_times_scalar_values += y_in[i] * scalar_values[i];
      sum_scalar_values += scalar_values[i];
    }

    std::pair<double, double> centroid = std::make_pair(sum_x_in_times_scalar_values / sum_scalar_values, sum_y_in_times_scalar_values / sum_scalar_values);

    // Compute the centroid without neighbors
    double sum_x_in_no_neigh_times_scalar_values = 0.0;
    double sum_y_in_no_neigh_times_scalar_values = 0.0;
    double sum_scalar_values_no_neigh = 0.0;


    for (size_t i = 0; i < x_in_no_neigh.size(); ++i)
    {
      sum_x_in_no_neigh_times_scalar_values += x_in_no_neigh[i] * scalar_values_no_neigh[i];
      sum_y_in_no_neigh_times_scalar_values += y_in_no_neigh[i] * scalar_values_no_neigh[i];
      sum_scalar_values_no_neigh += scalar_values_no_neigh[i];
    }


    std::pair<double, double> centroid_no_neigh = std::make_pair(sum_x_in_no_neigh_times_scalar_values / sum_scalar_values_no_neigh, sum_y_in_no_neigh_times_scalar_values / sum_scalar_values_no_neigh);


    // Compute the centroid without conn
    double sum_x_in_no_conn_times_scalar_values = 0.0;
    double sum_y_in_no_conn_times_scalar_values = 0.0;
    double sum_scalar_values_no_conn = 0.0;

    for (size_t i = 0; i < x_in_no_conn.size(); ++i)
    {
      sum_x_in_no_conn_times_scalar_values += x_in_no_conn[i] * scalar_values_no_conn[i];
      sum_y_in_no_conn_times_scalar_values += y_in_no_conn[i] * scalar_values_no_conn[i];
      sum_scalar_values_no_conn += scalar_values_no_conn[i];
    }

    std::pair<double, double> centroid_no_conn = std::make_pair(sum_x_in_no_conn_times_scalar_values / sum_scalar_values_no_conn, sum_y_in_no_conn_times_scalar_values / sum_scalar_values_no_conn);
    
    
    
    return std::make_tuple(centroid, centroid_no_neigh, centroid_no_conn);
  }

  void RBLController::apply_rules(double &beta,
                                             const std::vector<double> &c1,
                                             const std::vector<double> &c2,
                                             const std::vector<double> &current_position,
                                             double dt,
                                             double beta_min,
                                             const double &betaD,
                                             std::vector<double> &goal,
                                             double d1,
                                             double &th,
                                             double d2,
                                             double d3,
                                             double d4,
                                             std::pair<double, double> &destinations,
                                             std::vector<double> &c1_no_rotation)
  {

    // Extract x, y components from current_position
    double current_j_x = current_position[0];
    double current_j_y = current_position[1];

    // first condition
    
    double dist_c1_c2 = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2));
    if (dist_c1_c2 > d2 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d1)
    {
      beta = std::max(beta - dt, beta_min);
    }
    else
    {
      beta = beta - dt * (beta - betaD);
    }
    
    std::cout << "distc1_c2 = " << dist_c1_c2 <<"distp_c1 = " << sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) <<std::endl;
    
    // second condition
    bool dist_c1_c2_d4 = dist_c1_c2 > d4;
    if (dist_c1_c2_d4 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d3)
    {
      th = std::min(th + dt, M_PI / 2);
    }
    else
    {
      th = std::max(0.0, th - dt);
    }

    // third condition
    if (th == M_PI / 2 && sqrt(pow((current_j_x - c1_no_rotation[0]), 2) + pow((current_j_y - c1_no_rotation[1]), 2)) > sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)))
    {
      th = 0;
    }
    std::cout << "theta : "<< th<<", beta: "<< beta <<std::endl;
    // Compute the angle and new position
    double angle = atan2(goal[1] - current_j_y, goal[0] - current_j_x);
    double new_angle = angle - th;
    double distance = sqrt(pow((goal[0] - current_j_x), 2) + pow((goal[1] - current_j_y), 2));
    destinations.first = current_j_x + distance * cos(new_angle);
    destinations.second = current_j_y + distance * sin(new_angle);
  }

  void RBLController::getPositionCmd()
  {

    if (!is_initialized_)
    {
      return;
    }

    if (!sh_position_command_.hasMsg())
    {
      return;
    }

    mrs_msgs::TrackerCommand msg = *sh_position_command_.getMsg();

    geometry_msgs::PointStamped new_point;

    new_point.header = msg.header;
    new_point.point = msg.position;

    auto res = transformer_->transformSingle(new_point, _control_frame_);

    if (res)
    {
      new_point = res.value();
    }
    else
    {
      ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform position command to control frame.");
      return;
    }

    mrs_lib::set_mutexed(mutex_position_command_, new_point.point, position_command_);

    got_position_command_ = true;
  }

  //}

  /* odomCallback() //{ */
  void RBLController::odomCallback(const nav_msgs::OdometryConstPtr &msg, int idx)
  {

    if (!is_initialized_)
    {
      return;
    }

    /* nav_msgs::Odometry msg = *sh_ptr.getMsg(); */

    if ((ros::Time::now() - msg->header.stamp).toSec() > _odom_msg_max_latency_)
    {
      ROS_WARN("[RBLController]: The latency of odom message for %s exceeds the threshold (latency = %.2f s).", _uav_names_[idx].c_str(),
               (ros::Time::now() - msg->header.stamp).toSec());
    }

    geometry_msgs::PointStamped new_point;

    new_point.header = msg->header;
    new_point.point.x = msg->pose.pose.position.x;
    new_point.point.y = msg->pose.pose.position.y;
    new_point.point.z = msg->pose.pose.position.z;

    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res)
    {
      new_point = res.value();
    }
    else
    {
      ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform odometry msg to control frame.");
      return;
    }

    Eigen::Vector3d transformed_position;
    if (_c_dimensions_ == 3)
    {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    }
    else
    {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
    }

    mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_position, uav_positions_[idx]);

    last_odom_msg_time_[idx] = ros::Time::now();
  }
  //}

  /* callbackTimerSetVelocity() //{ */
  void RBLController::callbackTimerSetReference([[maybe_unused]] const ros::TimerEvent &te)
  {

    if (!is_initialized_)
    {
      return;
    }

    if (!all_robots_positions_valid_ && !got_position_command_)
    {
      ROS_WARN_THROTTLE(3.0, "[RBLController]: Waiting for valid robots' positions.");
      getPositionCmd();
      return;
    }

    if (!control_allowed_)
    {
      ROS_WARN_THROTTLE(3.0, "[RBLController]: Waiting for activation.");
      return;
    }

    getPositionCmd();

    /* TODO: calculate desired drone velocity or positional reference here */

    mrs_msgs::Reference p_ref;

    {

      std::vector<std::pair<double, double>> neighbors;

      std::scoped_lock lock(mutex_uav_odoms_, mutex_position_command_);
      robot_pos = {
          position_command_.x,
          position_command_.y,
      };

      for (int j = 0; j < n_drones_; ++j)
      {
        neighbors.push_back({uav_positions_[j][0], uav_positions_[j][1]});
      }

      std::vector<std::pair<double, double>> points = RBLController::points_inside_circle(robot_pos, radius, step_size);

      // Call find_closest_points function
      std::vector<std::pair<double, double>> closest_points = RBLController::find_closest_points(robot_pos, points, neighbors);

      // Call account_encumbrance function
      std::vector<std::pair<double, double>> accounted_points = RBLController::account_encumbrance(closest_points, robot_pos, neighbors, size_neighbors, encumbrance);

      // Call get_centroid function
      auto centroids = RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, encumbrance, destination, beta);

      auto centroids_no_rotation = RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, encumbrance, {goal[0], goal[1]}, beta);

      //std::vector<double> c1 = {c1_c2.first.first, c1_c2.first.second};
      //std::vector<double> c2 = {c1_c2.second.first, c1_c2.second.second};


      std::vector<double> c1 = {std::get<0>(centroids).first, std::get<0>(centroids).second};
      std::vector<double> c2 = {std::get<1>(centroids).first, std::get<1>(centroids).second};
      std::vector<double> c1_no_conn = {std::get<2>(centroids).first, std::get<2>(centroids).second};


      std::vector<double> c1_no_rotation = {std::get<2>(centroids_no_rotation).first, std::get<2>(centroids_no_rotation).second};
      
      std::vector<double> current_position(2); // Vector with two elements
      current_position[0] = robot_pos.first;   // Assigning first element
      current_position[1] = robot_pos.second;
      // std::vector<double> destinations = {destination.first, destination.second};
      

      // Call apply_rules function

      RBLController::apply_rules(beta, c1_no_conn, c2, current_position, dt, beta_min, betaD, goal, d1, th, d2, d3, d4, destination, c1_no_rotation);

      // double ts = 1.0 / double(_rate_timer_set_reference_);
      if (this_uav_idx_ < 7)
      {
        p_ref.position.x = c1[0]; // next_values[0];
        p_ref.position.y = c1[1];
        p_ref.position.z = 2.0;
      }
      else
      {
        p_ref.position.x = position_command_.x; // next_values[0];
        p_ref.position.y = position_command_.y;
        p_ref.position.z = 2.0;
      }
      // p_ref.heading = std::atan2(c1[1] - position_command_.y, c1[0] - position_command_.x);
    }

    ROS_INFO_THROTTLE(3.0, "[RBLController]: Setting positional reference [%.2f, %.2f, %.2f] in frame %s, heading = %.2f.", p_ref.position.x, p_ref.position.y, p_ref.position.z, _control_frame_.c_str(), p_ref.heading);

    // set drone velocity
    mrs_msgs::ReferenceStampedSrv srv;
    srv.request.reference = p_ref;
    srv.request.header.frame_id = _control_frame_;
    srv.request.header.stamp = ros::Time::now();

    if (sc_set_position_.call(srv))
    {
      ROS_INFO_THROTTLE(3.0, "Success: %d", srv.response.success);
      ROS_INFO_THROTTLE(3.0, "Message: %s", srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR_THROTTLE(3.0, "Failed to call service ref_pos_out");
    }
  }
  //}

  /* callbackTimerDiagnostics() //{ */
  void RBLController::callbackTimerDiagnostics([[maybe_unused]] const ros::TimerEvent &te)
  {

    if (!is_initialized_)
    {
      return;
    }

    bool timeout_exceeded = false;
    std::stringstream msg;
    msg.precision(2);
    msg << "Last odometry message received: ";
    for (int r = 0; r < last_odom_msg_time_.size(); r++)
    {
      double time_since_last_message = (ros::Time::now() - last_odom_msg_time_[r]).toSec();
      msg << _uav_names_[r] << " (" << time_since_last_message << " s), ";
      if (time_since_last_message > _odom_timeout_)
      {
        /* ROS_WARN("[RBLController]: Did not receive any message from %s for %.2f s.", _uav_names_[r].c_str(), time_since_last_message); */
        timeout_exceeded = true;
      }
    }

    ROS_WARN_COND(timeout_exceeded, "[RBLController]: %s", msg.str().c_str());

    all_robots_positions_valid_ = !timeout_exceeded;
  }
  //}

  /* activationServiceCallback() //{ */
  bool RBLController::activationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    // service for activation of planning
    ROS_INFO("[RBLController]: Activation service called.");
    res.success = true;

    if (control_allowed_)
    {
      res.message = "Control was already allowed.";
      ROS_WARN("[RBLController]: %s", res.message.c_str());
    }
    else if (!all_robots_positions_valid_)
    {
      res.message = "Robots are not ready, control cannot be activated.";
      ROS_WARN("[RBLController]: %s", res.message.c_str());
      res.success = false;
    }
    else
    {
      control_allowed_ = true;
      res.message = "Control allowed.";
      ROS_INFO("[RBLController]: %s", res.message.c_str());
    }

    return res.success;
  }

  //}

  /* SUPPORT FUNCTIONS //{ */

  //}

  // | -------------------- nodelet macro ----------------------- |
  PLUGINLIB_EXPORT_CLASS(formation_control::RBLController, nodelet::Nodelet);
} // namespace formation_control
