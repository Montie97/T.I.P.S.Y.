#include <ros/ros.h>
#include <move_base/move_base.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tipsy/Coordinates.h>
#include <tipsy/MovementStatus.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <std_msgs/String.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// C++ standard headers
#include <exception>
#include <string>
#include <list>

#define HIGH_TORSO 0.35
#define MID_TORSO 0.2
#define LOW_TORSO 0.0

#define GRIPPER_CLOSED 0.0
#define GRIPPER_OPEN 1.0

#define ARM_FRONT 1.57
#define ARM_SIDE 0.0

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

//for torso
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_control_client;
typedef boost::shared_ptr< torso_control_client>  torso_control_client_Ptr;

//for head
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;
typedef boost::shared_ptr< head_control_client>  head_control_client_Ptr;

//for gripper  
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef boost::shared_ptr< gripper_control_client>  gripper_control_client_Ptr;

ros::Time last_timestamp;

void createArmClient(arm_control_client_Ptr& actionClient)
{
    ROS_INFO("Creating action client to arm controller ...");
    actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );
    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
    }

    if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

//gripper
void createGripperClient(gripper_control_client_Ptr& actionClient)
{
    ROS_INFO("Creating action client to gripper controller ...");

    actionClient.reset( new gripper_control_client("gripper_controller/follow_joint_trajectory") );

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
    ROS_DEBUG("Waiting for the gripper_controller_action server to come up");
    ++iterations;
    }
    if ( iterations == max_iterations )
    throw std::runtime_error("Error in createGripperClient: gripper controller action server not available");
}

//torso
void createTorsoClient(torso_control_client_Ptr& actionClient)
{
    ROS_INFO("Creating action client to torso controller ...");
    actionClient.reset( new torso_control_client("/torso_controller/follow_joint_trajectory") );
    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
    ROS_DEBUG("Waiting for the torso_controller_action server to come up");
    ++iterations;
    }

    if ( iterations == max_iterations )
    throw std::runtime_error("Error in createTorsoClient: torso controller action server not available");
}

//head
void createHeadClient(head_control_client_Ptr& actionClient)
{
    ROS_INFO("Creating action client to head controller ...");
    actionClient.reset( new head_control_client("/head_controller/follow_joint_trajectory") );
    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
    ROS_DEBUG("Waiting for the head_controller_action server to come up");
    ++iterations;
    }

    if ( iterations == max_iterations )
    throw std::runtime_error("Error in createHeadClient: head controller action server not available");
}

//gripper 
void waypoints_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal, float gripper_pos)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    // One waypoint in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(2);
    goal.trajectory.points[index].positions[0] =  gripper_pos;
    goal.trajectory.points[index].positions[1] =  gripper_pos;
    // Velocities
    goal.trajectory.points[index].velocities.resize(2);
    goal.trajectory.points[index].velocities[0] = 0.0;
    goal.trajectory.points[index].velocities[1] = 0.0;

    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}

// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_arm_goal1(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("arm_1_joint");
    goal.trajectory.joint_names.push_back("arm_2_joint");
    goal.trajectory.joint_names.push_back("arm_3_joint");
    goal.trajectory.joint_names.push_back("arm_4_joint");
    goal.trajectory.joint_names.push_back("arm_5_joint");
    goal.trajectory.joint_names.push_back("arm_6_joint");
    goal.trajectory.joint_names.push_back("arm_7_joint");
    // One waypoint in this goal trajectory
    goal.trajectory.points.resize(1);
    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(7);
    goal.trajectory.points[index].positions[0] =  0.0;
    goal.trajectory.points[index].positions[1] = 1.1;
    goal.trajectory.points[index].positions[2] =  0.0;
    goal.trajectory.points[index].positions[3] =  0.0;
    goal.trajectory.points[index].positions[4] =  0.0;
    goal.trajectory.points[index].positions[5] =  0.0;
    goal.trajectory.points[index].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[index].velocities.resize(7);
    for (int j = 0; j < 7; ++j)
    {
    goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}

void waypoints_arm_goal2(control_msgs::FollowJointTrajectoryGoal& goal, float arm_pos)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("arm_1_joint");
    goal.trajectory.joint_names.push_back("arm_2_joint");
    goal.trajectory.joint_names.push_back("arm_3_joint");
    goal.trajectory.joint_names.push_back("arm_4_joint");
    goal.trajectory.joint_names.push_back("arm_5_joint");
    goal.trajectory.joint_names.push_back("arm_6_joint");
    goal.trajectory.joint_names.push_back("arm_7_joint");
    // One waypoint in this goal trajectory
    goal.trajectory.points.resize(1);
    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(7);
    goal.trajectory.points[index].positions[0] = arm_pos; //1.57
    goal.trajectory.points[index].positions[1] = -1.5;
    goal.trajectory.points[index].positions[2] = -3.14;
    goal.trajectory.points[index].positions[3] = 2.356;
    goal.trajectory.points[index].positions[4] = 1.57;
    goal.trajectory.points[index].positions[5] = 0.78;
    goal.trajectory.points[index].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[index].velocities.resize(7);
    for (int j = 0; j < 7; ++j)
    {
    goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}

//torso 
void waypoints_torso_goal(control_msgs::FollowJointTrajectoryGoal& goal, float torso_position)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    // One waypoint in this goal trajectory
    goal.trajectory.points.resize(1);
    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(1);
    goal.trajectory.points[index].positions[0] =  torso_position;
    // Velocities
    goal.trajectory.points[index].velocities.resize(1);
    goal.trajectory.points[index].velocities[0] = 0.0;

    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
}

//head 
void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");
    // One waypoint in this goal trajectory
    goal.trajectory.points.resize(1);
    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(2);
    goal.trajectory.points[index].positions[0] =  0.0;
    goal.trajectory.points[index].positions[1] =  -0.45;
    // Velocities
    goal.trajectory.points[index].velocities.resize(2);
    goal.trajectory.points[index].velocities[0] = 0.0;
    goal.trajectory.points[index].velocities[1] = 0.0;

    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}

void move_gripper(float gripper_pos)
{
    // Create an gripper controller action 
    gripper_control_client_Ptr GripperClient;
    createGripperClient(GripperClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal gripper_goal;
    waypoints_gripper_goal(gripper_goal, gripper_pos);
    // Sends the command to start the given trajectory 1s from now
    gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    GripperClient->sendGoal(gripper_goal);
    // Wait for trajectory execution
    while(!(GripperClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
}

void move_to_goal(tipsy::Coordinates coord, bool serving)
{
    // create the action client (true causes the client to spin its own thread)
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    ROS_INFO("COORDINATES: %f %f\n", coord.x, coord.y);

    goal.target_pose.header.stamp=ros::Time::now();
    // reference frame is map
    goal.target_pose.header.frame_id="map";
    // set goal as 2D coordinate
    goal.target_pose.pose.position.x=coord.x;
    goal.target_pose.pose.position.y=coord.y;
    // set goal orientation
    goal.target_pose.pose.orientation.x=0.0;
    goal.target_pose.pose.orientation.y=0.0;
    if(serving){
        goal.target_pose.pose.orientation.z=0.0;
        goal.target_pose.pose.orientation.w=1.0;
    }
    else{
        goal.target_pose.pose.orientation.z=1.0;
        goal.target_pose.pose.orientation.w=0.0;
    }

    ROS_INFO("Sending goal.");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Movement goal reached!");
    else
        ROS_INFO("Movement failed.");
}

// I probably won't need this
void send_ack()
{
    ros::NodeHandle n; // SHOULD I DO THIS?
    ros::Rate loop_rate(1); // AND THIS?
    tipsy::MovementStatus move_status;
    move_status.finished = true;
    ros::Publisher ack_pub = n.advertise<tipsy::Coordinates>("ack_topic", 10);
    while (ros::ok()) {
        ack_pub.publish(move_status);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void move_torso(float torso_position)
{
    // Create an torso controller action 
    torso_control_client_Ptr TorsoClient;
    createTorsoClient(TorsoClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal torso_goal;
    waypoints_torso_goal(torso_goal, torso_position);
    // Sends the command to start the given trajectory 1s from now
    torso_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    TorsoClient->sendGoal(torso_goal);
    // Wait for trajectory execution
    while(!(TorsoClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
}

void eyes_on_the_prize()
{
    // Create an head controller action 
    head_control_client_Ptr HeadClient;
    createHeadClient(HeadClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal head_goal;
    waypoints_head_goal(head_goal);
    // Sends the command to start the given trajectory 1s from now
    head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    HeadClient->sendGoal(head_goal);
    // Wait for trajectory execution
    while(!(HeadClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
}

void arm_grabbing_position(float arm_pos)
{
    // Create an arm controller action client to move the TIAGo's arm
    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal arm_goal2;
    waypoints_arm_goal2(arm_goal2, arm_pos);
    // Sends the command to start the given trajectory 1s from now
    arm_goal2.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    ArmClient->sendGoal(arm_goal2);
    // Wait for trajectory execution
    while(!(ArmClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
}

void raise_arm()
{
    // Create an arm controller action client to move the TIAGo's arm
    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal arm_goal1;
    waypoints_arm_goal1(arm_goal1);
    // Sends the command to start the given trajectory 1s from now
    arm_goal1.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    ArmClient->sendGoal(arm_goal1);
    // Wait for trajectory execution
    while(!(ArmClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
}

// CALLBACK FUNCTION
void serve_drink(const tipsy::Coordinates::ConstPtr &p_coord) 
{  
    tipsy::Coordinates coords;
    bool serving = true;
    if(p_coord->timestamp > last_timestamp){
        last_timestamp = p_coord->timestamp;
        ROS_INFO("Entering callback function\n");
        raise_arm();
        move_to_goal(*p_coord, !serving);
        eyes_on_the_prize();
        move_torso(HIGH_TORSO); 
        arm_grabbing_position(ARM_FRONT);
        move_torso(MID_TORSO);
        move_gripper(GRIPPER_CLOSED);
        arm_grabbing_position(ARM_SIDE);
        // Move a little away from the table
        //coords.x = p_coord->x;
        //coords.y = p_coord->y + 0.5;
        //move_to_goal(coords, serving);
        move_torso(LOW_TORSO);
        // Move to the serving table
        coords.x = 0.3;
        coords.y = 0.5;
        move_to_goal(coords, serving);
        move_torso(HIGH_TORSO);
        arm_grabbing_position(ARM_FRONT);
        move_gripper(GRIPPER_OPEN);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    last_timestamp = ros::Time::now();

    ros::Subscriber sub = n.subscribe("coordinates_topic", 1, serve_drink);
    ros::spin();
    //loop_rate.sleep();

    return 0;
}