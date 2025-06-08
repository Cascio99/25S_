#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gps_publisher");
    ros::NodeHandle nh;

    // Publishers for /gps_data and /waypoints
    ros::Publisher gps_pub = nh.advertise<std_msgs::String>("/gps_data", 1, true);
    ros::Publisher waypoints_pub = nh.advertise<std_msgs::Float64MultiArray>("/waypoints", 1, true);

    // Define waypoints
    std::vector<std::pair<double, double>> waypoints = {
        {35.846087646108636, 127.13440823476309},   // initial position
        {35.8461, 127.1344},
        {35.8461, 127.1346},
        {35.8461, 127.1348},
        {35.8461, 127.1350},
        {35.8461, 127.1352}
    };

    // Publish /gps_data (initial GPS data)
    std_msgs::String gps_msg;
    std::ostringstream gps_data;
    gps_data << waypoints[0].first << "," << waypoints[0].second; // Use the first waypoint as initial GPS
    gps_msg.data = gps_data.str();
    gps_pub.publish(gps_msg);
    ROS_INFO("Published initial GPS data: %s", gps_msg.data.c_str());

    // Publish /waypoints
    std_msgs::Float64MultiArray waypoints_msg;
    for (const auto& wp : waypoints) {
        waypoints_msg.data.push_back(wp.first);  // Latitude
        waypoints_msg.data.push_back(wp.second); // Longitude
    }
    waypoints_pub.publish(waypoints_msg);
    ROS_INFO("Published waypoints.");

    ros::spin(); // Keep the node alive
    return 0;
}