#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <GeographicLib/UTMUPS.hpp> // Include GeographicLib for UTM conversion

ros::Publisher fused_pub;

// Global variables for UTM origin
static double utm_origin_x = 0.0;
static double utm_origin_y = 0.0;
static bool origin_set = false;
static bool initial_published = false;

// Global variables for latest UTM coordinates
static double latest_utm_x = 0.0;
static double latest_utm_y = 0.0;

void gpsCallback(const std_msgs::String::ConstPtr& msg) {
    if (origin_set) return; // Run only once

    // Parse GPS data (comma-separated string)
    std::istringstream gps_stream(msg->data);
    std::string latitude_str, longitude_str;
    if (std::getline(gps_stream, latitude_str, ',') && std::getline(gps_stream, longitude_str, ',')) {
        double latitude = std::stod(latitude_str);
        double longitude = std::stod(longitude_str);

        // Convert WGS84 to UTM
        int zone;
        bool northp;
        double utm_x, utm_y;
        GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_x, utm_y);

        // Set the UTM origin
        utm_origin_x = utm_x;
        utm_origin_y = utm_y;

        // Update the latest UTM coordinates
        latest_utm_x = utm_x;
        latest_utm_y = utm_y;

        origin_set = true;

        ROS_INFO("Initial GPS data received and UTM origin set: x=%f, y=%f", utm_origin_x, utm_origin_y);
    } else {
        ROS_WARN("Failed to parse GPS data.");
    }
}

void wpCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    static bool waypoints_received = false;
    if (waypoints_received) return; // Run only once

    // Process waypoints data (if needed)
    ROS_INFO("Waypoints data received.");
    waypoints_received = true;
}

void lioCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("lioCallback triggered with data: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);

    // Fuse LIO odometry with GPS data
    nav_msgs::Odometry fused_msg = *msg;
    fused_msg.pose.pose.position.x += latest_utm_x;  // Use the global latest UTM x-coordinate
    fused_msg.pose.pose.position.y += latest_utm_y;  // Use the global latest UTM y-coordinate

    ROS_INFO("Fused data: x=%f, y=%f", fused_msg.pose.pose.position.x, fused_msg.pose.pose.position.y);

    // Publish fused data
    fused_pub.publish(fused_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "GPP_LIO_Integration");
    ros::NodeHandle nh;
    
    // Subscribe to GPP and Faster-LIO topics
    ros::Subscriber gps_sub = nh.subscribe("/gps_data", 10, gpsCallback);
    ros::Subscriber wp_sub = nh.subscribe("/waypoints", 10, wpCallback);
    ros::Subscriber lio_sub = nh.subscribe("/Odometry", 10, lioCallback);

    // Publisher for utm position
    fused_pub = nh.advertise<nav_msgs::Odometry>("/utm_odom", 10);

    ros::spin();

    return 0;
}