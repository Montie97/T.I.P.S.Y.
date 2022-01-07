#include <iostream>
#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>

void print_aruco(const aruco_msgs::MarkerArray::ConstPtr &p_marker) 
{  
    ROS_INFO("MARKER X Y id: %f %f \n", p_marker->markers[0].pose.pose.position.x, p_marker->markers[0].pose.pose.position.y);
    float marker_x;
    float marker_y;
    float desired_marker_x;
    float desired_marker_y;

    marker_x = p_marker->markers[0].pose.pose.position.x;
    marker_y = p_marker->markers[0].pose.pose.position.y;

    //with these coordinates the grabbing works perfectly
    desired_marker_x = 0.7456;
    desired_marker_y = 0.0473;

    //we want to move tiago so that marker_x = desired_marker_x
    //x_tiago_new = x_tiago_old + (desired_marker_x - marker_x);  
    //y_tiago_new = y_tiago_old + (desired_marker_y - marker_y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    ros::Subscriber sub = n.subscribe("aruco_marker_publisher/markers", 10, print_aruco);

    ros::spin();
    //loop_rate.sleep();

    return 0;
}