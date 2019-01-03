#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define the pich up zone position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -3;
  goal.target_pose.pose.position.y = 5;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the pich up zone position and orientation for the robot to reach
  ROS_INFO("Sending pich up zone");
  ac.sendGoal(goal);

  // Wait 30 seconds for the results
  ac.waitForResult(ros::Duration(90.0));

  // Check if the robot reached pick up zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved to pick up zone");
    // Wait 5 sec then move to the drop off zone
    ros::Duration(5.0).sleep();

    // Define the drop off zone position and orientation for the robot to reach
    goal.target_pose.pose.position.x = 4.5;
    goal.target_pose.pose.position.y = 7;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the drop off zone position and orientation for the robot to reach
    ROS_INFO("Sending drop off zone");
    ac.sendGoal(goal);

    // Wait 30 seconds for the results
    ac.waitForResult(ros::Duration(90.0));

    // Check if the robot reached drop off zone
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base moved to drop off zone");
      ros::Duration(5.0).sleep();
    } 
    else ROS_INFO("The base failed to move to drop off zone");
  } 
  else ROS_INFO("The base failed to move to pick up zone");

  return 0;
}

