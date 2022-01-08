// C++ standard headers
#include <exception>
#include <string>
#include <list>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <std_msgs/String.h>
#include "tipsy/StartMsg.h"
#include "tipsy/ContinueMsg.h"


////////////////////////////////////////////////
///// A C T I O N  I N T E R F A C E ///////////

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

////////////////////////////////////////////////
///// A C T I O N  C L I E N T  ////////////////

/*arm
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
}*/
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


////////////////////////////////////////////////
/////////////////// G O A L  ///////////////////

/* Generates a simple trajectory with two waypoints to move TIAGo's arm 
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
void waypoints_arm_goal2(control_msgs::FollowJointTrajectoryGoal& goal)
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
  goal.trajectory.points[index].positions[0] = 1.57;
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
void waypoints_torso_goal1(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("torso_lift_joint");

  // One waypoint in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(1);
  goal.trajectory.points[index].positions[0] =  0.35;
  // Velocities
  goal.trajectory.points[index].velocities.resize(1);
  goal.trajectory.points[index].velocities[0] = 0.0;
  
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
}
void waypoints_torso_goal2(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("torso_lift_joint");

  // One waypoint in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(1);
  goal.trajectory.points[index].positions[0] =  0.2;
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
}*/

//gripper 
void waypoints_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
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
  goal.trajectory.points[index].positions[0] =  0.0;
  goal.trajectory.points[index].positions[1] =  0.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(2);
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;
  
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}
void waypoints_gripper_goal2(control_msgs::FollowJointTrajectoryGoal& goal)
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
  goal.trajectory.points[index].positions[0] =  1.0;
  goal.trajectory.points[index].positions[1] =  1.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(2);
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;
  
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}

////////////////////////////////////////////////
/////////////////// G O A L  ///////////////////

void grabbingPosition() 
{
    /* step 1 : ARM 
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

    // step 2: TORSO 
    // Create an torso controller action 
    torso_control_client_Ptr TorsoClient;
    createTorsoClient(TorsoClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal torso_goal;
    waypoints_torso_goal1(torso_goal);
    // Sends the command to start the given trajectory 1s from now
    torso_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    TorsoClient->sendGoal(torso_goal);
    // Wait for trajectory execution
    while(!(TorsoClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }

    // step 3: HEAD 
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

    // step 4: ARM 2 
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal arm_goal2;
    waypoints_arm_goal2(arm_goal2);
    // Sends the command to start the given trajectory 1s from now
    arm_goal2.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    ArmClient->sendGoal(arm_goal2);
    // Wait for trajectory execution
    while(!(ArmClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    } 

    //Step 5: torso 2
    
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal torso_goal2;
    waypoints_torso_goal2(torso_goal2);
    // Sends the command to start the given trajectory 1s from now
    torso_goal2.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    TorsoClient->sendGoal(torso_goal2);
    // Wait for trajectory execution
    while(!(TorsoClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }*/
    
    // step 6: gripper 
    // Create an gripper controller action 
    gripper_control_client_Ptr GripperClient;
    createGripperClient(GripperClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal gripper_goal;
    waypoints_gripper_goal(gripper_goal);
    // Sends the command to start the given trajectory 1s from now
    gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    GripperClient->sendGoal(gripper_goal);
    // Wait for trajectory execution
    while(!(GripperClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
    // step 7: gripper RELEAAAAAAASE ME  
    // Create an gripper controller action 
    //gripper_control_client_Ptr GripperClient;
    //createGripperClient(GripperClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal gripper_goal2;
    waypoints_gripper_goal2(gripper_goal2);
    // Sends the command to start the given trajectory 1s from now
    gripper_goal2.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    GripperClient->sendGoal(gripper_goal2);
    // Wait for trajectory execution
    while(!(GripperClient->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
    return;
}
////////////////////////////////////////////////
////// C A L L B A C K  F U N C T I O N ////////

void callbackSubs (const tipsy::StartMsg::ConstPtr &msg) 
{
    auto bob = msg->okToStart;
    switch(bob)
    {
        case 1:
        {
            ROS_INFO("Robot in the right position.");
            grabbingPosition();
            break;
        }
        default :
        {
            ROS_INFO("Robot not in the right position, grabbing or releasing couldn't start.");
            break;
        }
    }
return;
}

////////////////////////////////////////////////
/////////////////// M A I N ////////////////////

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_traj_control");

  ROS_INFO("Starting run_traj_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  ROS_INFO("Subscriber acting");
  ros::Subscriber sub = nh.subscribe("chatter", 1000, callbackSubs );
  ros::spinOnce();

  ros::Publisher pub=nh.advertise<tipsy::ContinueMsg>("chatterPub",1000);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    tipsy::ContinueMsg msge;
    msge.okToContinue=1;
    ROS_INFO("Set value");
    ros::Duration(4).sleep();
    pub.publish(msge);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}