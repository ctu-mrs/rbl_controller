#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <unordered_set>

class MarkerReset
{
public:
    MarkerReset()
    {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Set up a publisher to the MarkerArray topic
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/uav33/rbl_controller/neighbors_vis", 10);

        // Set up a timer to reset markers every second
        timer = nh.createTimer(ros::Duration(1.0), &MarkerReset::resetMarkers, this);

        // Subscribe to the same topic to track marker IDs
        marker_sub = nh.subscribe("/uav33/rbl_controller/neighbors_vis", 10, &MarkerReset::markerCallback, this);
    }

    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
    {
        // Loop through the received markers and track their IDs
        for (const auto& marker : msg->markers)
        {
            marker_ids.insert(marker.id); // Insert marker ID into the set
        }
    }

    void resetMarkers(const ros::TimerEvent&)
    {
        // Create a MarkerArray to send DELETE actions
        visualization_msgs::MarkerArray delete_marker_array;

        // Loop over all tracked marker IDs and set the action to DELETE
        for (int id : marker_ids)
        {
            visualization_msgs::Marker marker;
            marker.id = id;
            marker.action = visualization_msgs::Marker::DELETE;
            delete_marker_array.markers.push_back(marker);
        }

        // Publish the delete action to reset markers
        if (!delete_marker_array.markers.empty())
        {
            marker_pub.publish(delete_marker_array);
            ROS_INFO("Publishing DELETE actions to reset markers.");
        }

        // Optionally, clear the marker_ids set to reset for the next cycle
        marker_ids.clear();
    }

private:
    ros::Publisher marker_pub;
    ros::Subscriber marker_sub;
    ros::Timer timer;
    std::unordered_set<int> marker_ids;  // Set of dynamic marker IDs to reset (delete)
};

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "marker_reset_node");

    // Create an instance of MarkerReset
    MarkerReset marker_reset;

    // Spin the ROS node to keep it running
    ros::spin();

    return 0;
}
