/*
 * This program is used to synchronize the vision of a person and a robot.
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

#include <geometry_msgs/PoseStamped.h>
#include <iostream>
using namespace std;

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;
typedef boost::shared_ptr< head_control_client>  head_control_client_Ptr;

// Create a head controller action client to move the TIAGo's head
head_control_client_Ptr head_client;

// Create a ROS action client to move TIAGo's arm
void createHeadClient(head_control_client_Ptr& action_client, const std::string arm_controller_name)
{
  ROS_INFO("Creating action client to %s ...", arm_controller_name.c_str());

  std::string action_client_name = "/" + arm_controller_name + "/follow_joint_trajectory";
  action_client.reset( new head_control_client(action_client_name) );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createHeadClient: head controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's right arm 
void head_sync(control_msgs::FollowJointTrajectoryGoal& goal, double roll, double yaw)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("head_1_joint");
  goal.trajectory.joint_names.push_back("head_2_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(2);

  if(roll>1.24){
    roll = 1.24;
  }
  if(roll<-1.24){
    roll = -1.24;
  }
  if(yaw>0.72){
    yaw = 0.72;
  }
  if(yaw<-0.98){
    yaw = -0.98;
  }
  goal.trajectory.points[0].positions[0] = -yaw; // Right(-) and Left(+) -1.24 <= x <= 1.24 
  goal.trajectory.points[0].positions[1] = roll; // Down(-) and Up(+) -0.98 <= x <= 0.72

  // Velocities
  goal.trajectory.points[0].velocities.resize(2);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[0].velocities[j] = 1.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
}

void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double& roll, double& pitch, double& yaw)
{
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

void sync_cb(geometry_msgs::PoseStamped msg){

    double roll,pitch,yaw;

    double x = msg.pose.orientation.x;
    double y = msg.pose.orientation.y;
    double z = msg.pose.orientation.z;
    double w = msg.pose.orientation.w;

    QuaternionToEulerAngles(w,x,y,z,roll,pitch,yaw);

    cout << "x:" << x << "\n";
    cout << "y:" << y << "\n";
    cout << "z:" << z << "\n";
    cout << "w:" << w << "\n";
    cout << "roll:" << roll << "\n";
    cout << "pitch:" << pitch << "\n";
    cout << "yaw:" << yaw << "\n";
    cout << "-----------------------------------\n";

    
    // Generates the goal for the TIAGo's head
    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_sync(head_goal,roll,yaw);

    // // Sends the command to start the given trajectory 0.005s from now
    head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.005);
    head_client->sendGoal(head_goal);
    

    return;

}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "vr_sync");

  ROS_INFO("Sync Start VR->HEAD");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }
  createHeadClient(head_client, "head_controller");



  ros::Subscriber sub = nh.subscribe("/unity/rot", 1, sync_cb);

  ros::spin();


  return 0;
}

