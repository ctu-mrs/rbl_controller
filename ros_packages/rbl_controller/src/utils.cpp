nav_msgs::Path WrapperRBL::getVizPath(const std::vector<Eigen::Vector3d>& path, const std::string& frame) {
  if (path.empty()) {
    ROS_WARN("getVizPath: Received empty path.");
    return;
  }

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = frame;

  for (const auto& pt : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame;
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

  return path_msg;
}

visualization_msgs::MarkerArray WrapperRBL::getVizPlanes(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& planes, const std::string& frame) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(planes.size());

  for (size_t i = 0; i < planes.size(); ++i) {
    const Eigen::Vector3d& normal = planes[i].first;
    const Eigen::Vector3d& point = planes[i].second;
    visualization_msgs::Marker& marker = marker_array.markers[i];
    marker.header.frame_id = frame; // Adjust frame ID as needed
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

  return marker_array;
}

std::shared_ptr<sensor_msgs::PointCloud2> WrapperRBL::getVizCellA(const std::vector<Eigen::Vector3d>& points, const std::string& frame) {
  pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
  tmp_pcl.width = points.size();
  tmp_pcl.height = 1;
  tmp_pcl.is_dense = true;
  tmp_pcl.points.resize(tmp_pcl.width * tmp_pcl.height);

  for (size_t i = 0; i < points.size(); ++i) {
    tmp_pcl.points[i].x = static_cast<float>(points[i].x());
    tmp_pcl.points[i].y = static_cast<float>(points[i].y());
    tmp_pcl.points[i].z = static_cast<float>(points[i].z());
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(tmp_pcl, output);
  output.header.frame_id = frame;
  output.header.stamp = ros::Time::now();

return std::make_shared<sensor_msgs::PointCloud2>(output);
}
//}


  visualization_msgs::Marker            getVizPosition(const Eigen::Vector3d& point, const double scale, const std::string& frame){
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "position";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = point(0);
  marker.pose.position.y    = point(1);
  marker.pose.position.z    = point(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 0.0;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;
}

  visualization_msgs::Marker            getVizCentroid(const Eigen::Vector3d& point, const std::string& frame){
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "centroid";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = point(0);
  marker.pose.position.y    = point(1);
  marker.pose.position.z    = point(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.2;
  marker.scale.y            = 0.2;
  marker.scale.z            = 0.2;
  marker.color.r            = 1.0;
  marker.color.g            = 0.0;
  marker.color.b            = 0.0;
  marker.color.a            = 1.0;
}

  visualization_msgs::Marker            getVizDestination(const Eigen::Vector3d& point, const double scale, const std::string& frame) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "destination";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = point(0);
  marker.pose.position.y    = point(1);
  marker.pose.position.z    = point(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 0.0;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;
}
//}

std::shared_ptr<sensor_msgs::PointCloud2> WrapperRBL::getVizPcl(const pcl::PointCloud<pcl::PointXYZ>& pcl, const std::string& frame) {
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pcl, output);
  output.header.frame_id = frame;
  output.header.stamp = ros::Time::now();

return std::make_shared<sensor_msgs::PointCloud2>(output);
}

visualization_msgs::Marker WrapperRosRBL::getVizConnection(const Eigen::Vector3d& target_point, const Eigen::Vector3d& position, cosnt std::string& frame) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame; // Use the provided frame ID
  marker.header.stamp = ros::Time::now();
  marker.ns = "target";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;

  // Set marker scale (length and thickness of the arrow)
  marker.scale.x = 0.1; // is the shaft diameter
  marker.scale.y = 0.15; //is the head diameter.
  marker.scale.z = 0.2; // is not zero, it specifies the head length.

  // Set marker color
  marker.color.r = 1.0f; // Red
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f; // Opaque

  // Set the end point of the arrow (vector direction)
  marker.pose.orientation.w = 1;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;

  marker.points.resize(2);
  marker.points[0].x = position(0);
  marker.points[0].y = position(1);
  marker.points[0].z = position(2);

  marker.points[1].x = target_point(0);
  marker.points[1].y = target_point(1);
  marker.points[1].z = target_point(2);

  marker.lifetime = ros::Duration(); // Infinite lifetime

return marker;
}

std::vector<Eigen::Vector3d> WrapperRosRBL::pointCloudToEigenVector(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.reserve(cloud.size());

  for (const auto& pt : cloud.points) {
    eigen_points.emplace_back(pt.x, pt.y, pt.z);
  }

  return eigen_points;
}
