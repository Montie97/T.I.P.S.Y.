#include <ros/ros.h>
#include <move_base/move_base.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef struct point{
    float x;
    float y;
} goal_point;

move_base_msgs::MoveBaseGoal set_goal(goal_point p) 
{  
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp=ros::Time::now();
    // reference frame is map
    goal.target_pose.header.frame_id="map";
    // set goal as 2D coordinate
    goal.target_pose.pose.position.x=p.x;
    goal.target_pose.pose.position.y=p.y;
    // set goal orientation
    goal.target_pose.pose.orientation.x=0.0;
    goal.target_pose.pose.orientation.y=0.0;
    goal.target_pose.pose.orientation.z=0.0;
    goal.target_pose.pose.orientation.w=1.0;

    return goal;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiago_patrol_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

    goal_point pointA, pointB, pointC;
    pointA.x = -1.0;
    pointA.y = 0.0;
    pointB.x = 0.0;
    pointB.y = -1.0;
    pointC.x = 1.0;
    pointC.y = 1.0;
    
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    //wait for the action server to come up
    /*while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }*/
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    while(ros::ok()){

        // POINT B

        move_base_msgs::MoveBaseGoal goal = set_goal(pointB);
        ROS_INFO("Sending goal.");
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Movement goal reached!");
        else
            ROS_INFO("Movement failed.");

        // POINT A

        goal = set_goal(pointA);
        ROS_INFO("Sending goal.");
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Movement goal reached!");
        else
            ROS_INFO("Movement failed.");

        // POINT C

        goal = set_goal(pointC);
        ROS_INFO("Sending goal.");
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Movement goal reached!");
        else
            ROS_INFO("Movement failed.");

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}