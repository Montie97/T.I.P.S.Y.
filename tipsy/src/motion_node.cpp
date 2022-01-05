#include <ros/ros.h>
#include <move_base/move_base.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tipsy/Coordinates.h>

void reach_goal(const tipsy::Coordinates::ConstPtr &p_coord) 
{  
    // create the action client (true causes the client to spin its own thread)
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    ROS_INFO("COORDINATES: %f %f %f\n", p_coord->x, p_coord->y, p_coord->Y);

    goal.target_pose.header.stamp=ros::Time::now();
    // reference frame is map
    goal.target_pose.header.frame_id="map";
    // set goal as 2D coordinate
    goal.target_pose.pose.position.x=p_coord->x;
    goal.target_pose.pose.position.y=p_coord->y;
    // set goal orientation
    goal.target_pose.pose.orientation.x=0.0;
    goal.target_pose.pose.orientation.y=0.0;
    goal.target_pose.pose.orientation.z=0.0;
    goal.target_pose.pose.orientation.w=1.0;

    ROS_INFO("Sending goal.");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Movement goal reached!");
    else
        ROS_INFO("Movement failed.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    //wait for the action server to come up
    /*while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }*/

    ros::Subscriber sub = n.subscribe("coordinates_topic", 10, reach_goal);

    ros::spin();
    //loop_rate.sleep();

    return 0;
}