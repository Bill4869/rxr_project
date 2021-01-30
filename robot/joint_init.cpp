/*
 * This program is used to initialize the positions of arms. (fold arms to avoid collisions with object while the operation.)
 */


// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& action_client, const std::string arm_controller_name)
{
  ROS_INFO("Creating action client to %s ...", arm_controller_name.c_str());

  std::string action_client_name = "/" + arm_controller_name + "/follow_joint_trajectory";
  action_client.reset( new arm_control_client(action_client_name) );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypointsArmGoal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(7);
  goal.trajectory.points[0].positions[0] = 1.50079632679;
  goal.trajectory.points[0].positions[1] = 1.50079632679;
  goal.trajectory.points[0].positions[2] = 0.0;
  goal.trajectory.points[0].positions[3] = 2.28619449019;
  goal.trajectory.points[0].positions[4] = 0.0;
  goal.trajectory.points[0].positions[5] = -1.39371669412;
  goal.trajectory.points[0].positions[6] = 0.0;

  // Velocities
  goal.trajectory.points[0].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[0].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
}


// Generates a simple trajectory with two waypoints to move TIAGo's left arm 
void waypointsArmLeftGoal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_left_1_joint");
  goal.trajectory.joint_names.push_back("arm_left_2_joint");
  goal.trajectory.joint_names.push_back("arm_left_3_joint");
  goal.trajectory.joint_names.push_back("arm_left_4_joint");
  goal.trajectory.joint_names.push_back("arm_left_5_joint");
  goal.trajectory.joint_names.push_back("arm_left_6_joint");
  goal.trajectory.joint_names.push_back("arm_left_7_joint");

  waypointsArmGoal(goal);
}


// Generates a simple trajectory with two waypoints to move TIAGo's right arm 
void waypointsArmRightGoal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_right_1_joint");
  goal.trajectory.joint_names.push_back("arm_right_2_joint");
  goal.trajectory.joint_names.push_back("arm_right_3_joint");
  goal.trajectory.joint_names.push_back("arm_right_4_joint");
  goal.trajectory.joint_names.push_back("arm_right_5_joint");
  goal.trajectory.joint_names.push_back("arm_right_6_joint");
  goal.trajectory.joint_names.push_back("arm_right_7_joint");

  waypointsArmGoal(goal);
}

// Generates a simple trajectory with two waypoints to move TIAGo's right arm 
void waypointsTorso(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("torso_lift_joint");
  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(1);
  goal.trajectory.points[0].positions[0] = 0;

  // Velocities
  goal.trajectory.points[0].velocities.resize(1);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[0].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
}


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "joint_init");

  ROS_INFO("Initializing the arm joint positions ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create an arm left controller action client to move the TIAGo's left arm
  arm_control_client_Ptr arm_left_client;
  createArmClient(arm_left_client, "arm_left_controller");

  // Create an arm right controller action client to move the TIAGo's right arm
  arm_control_client_Ptr arm_right_client;
  createArmClient(arm_right_client, "arm_right_controller");


  // Generates the goal for the TIAGo's left arm
  control_msgs::FollowJointTrajectoryGoal arm_left_goal;
  waypointsArmLeftGoal(arm_left_goal);

  // Sends the command to start the given trajectory 0.2s from now
  arm_left_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
  arm_left_client->sendGoal(arm_left_goal);

  // Wait for trajectory execution
  while(!(arm_left_client->getState().isDone()) && ros::ok())
  {
    ros::Duration(1).sleep(); // sleep for four seconds
  }

  // Generates the goal for the TIAGo's right arm
  control_msgs::FollowJointTrajectoryGoal arm_right_goal;
  waypointsArmRightGoal(arm_right_goal);

  // Sends the command to start the given trajectory 0.2s from now
  arm_right_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
  arm_right_client->sendGoal(arm_right_goal);

  ros::Duration(1).sleep();
  ROS_INFO("Initialization Signal has been sent.");

  // Wait for trajectory execution
  while(!(arm_right_client->getState().isDone()) && ros::ok())
  {
    ros::Duration(1).sleep(); // sleep for four seconds
  }

  return EXIT_SUCCESS;
}

