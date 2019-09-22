#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


move_base_msgs::MoveBaseGoal createGoal(double pos_x, double pos_y, double orientation_w){
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pos_x;
  goal.target_pose.pose.position.y = pos_y;
  goal.target_pose.pose.orientation.w = orientation_w;
  
  return goal;
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // create vector of goals
  std::vector<move_base_msgs::MoveBaseGoal> goals;
  goals.push_back(createGoal(1.0,1.0,1.0));
  //goals.push_back(createGoal(1.0,1.0,1.0));

  // reach all goals
  for(int i=0; i<goals.size(); ++i){
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goals[i]);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("The robot reached the goal");
      // wait 5 seconds
      sleep(5);
    }
    else
    {
      ROS_INFO("The robot failed to reach goal");
      return 1;
    }
  }
  
  // never leave the main
  while(true){}

  return 0;
}