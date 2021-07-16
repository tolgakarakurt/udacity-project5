#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* main function */
int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //set up publisher to broadcast if robot is at goal marker
  ros::Publisher goal_reach_pub = n.advertise<std_msgs::UInt8>("/goal_reached", 1);
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;		// pick-up & drop goal
  std_msgs::UInt8 status_msg;  // goal reach status

  // set up the frame parameters
  // pick-goal setup
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Send the goal position and orientation for the robot for pick-up location
  ROS_INFO("Publishing pick-up coordinates");

  goal.target_pose.pose.position.x = -2.0;
  goal.target_pose.pose.position.y = 0.5;
  goal.target_pose.pose.orientation.w = 1.0;
  
  //ROS_INFO("GOAL coordinates:", goal);
  ROS_INFO("Sending pick-up coordinates");
  // Wait an infinite time for the results

  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached pick-up zone");
    ROS_INFO("Picking up the package");
    // publish goal-reach status
    status_msg.data = 1;
    ros::Duration(5.0).sleep();
    // ROS_INFO("The pick-up goal-reach status is %d", status_msg.data);
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("The robot failed to reach pick-up zone");
    return 0;
  }


  // robot reached pick-up location, send drop-off location
  ROS_INFO("Sending drop-off goal");
  // wait a bit before next message
  ros::Duration(3.0).sleep();

  goal.target_pose.pose.position.x = -3.0;
  goal.target_pose.pose.position.y = -3.5;
  goal.target_pose.pose.orientation.w = 1.5;

  ac.sendGoal(goal);

  ROS_INFO("Moving to Drop-off site");
  // Wait an infinite time for the results
  ac.sendGoal(goal);

  ac.waitForResult();

  // Check if the robot reached its drop goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //ROS_INFO("Robot has reached DROP-OFF location");
    //sleep(2);
    ROS_INFO("Dropping Package");
    // publish goal-reach status
    status_msg.data = 3;
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("The robot failed to reach drop-off zone");
  }

  // wait a bit before next message
  ros::Duration(5.0).sleep();

  return 0;
}
