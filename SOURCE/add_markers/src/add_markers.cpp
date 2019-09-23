#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <queue>
#include <cmath>

geometry_msgs::Pose2D current_pose;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;
}

void initiateMarker(visualization_msgs::Marker& marker){
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "robotnd_udacity";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
}

#include <iostream>

double computeDistance(double x1, double y1, double x2, double y2){
  return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // add odometry subscriber
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);
  
  std::queue<std::vector<double> > goals;
  goals.push({-7.5,-7.0,1.0});
  goals.push({7.5,-7.0,1.0});
  
  visualization_msgs::Marker marker;
  initiateMarker(marker);
  
  ROS_INFO_ONCE("Publish the marker at the pickup zone");
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = goals.front()[0];
  marker.pose.position.y = goals.front()[1];
  
  bool pickup = true;
  
  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    const double dist = computeDistance(goals.front()[0],goals.front()[1],current_pose.x, current_pose.y);
    
    if(pickup){
      ROS_INFO_ONCE("Driving to the first goal");
      marker_pub.publish(marker);
      if(dist < 1){
        ROS_INFO_ONCE("Hide the marker");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        
        goals.pop();     
        pickup = false;
      }
    }
    else{
      ROS_INFO_ONCE("Driving to the drop off zone");
      if(dist < 1){
        ROS_INFO_ONCE("Publish the marker at the drop off zone");
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goals.front()[0];
        marker.pose.position.y = goals.front()[1];
        marker_pub.publish(marker);
        return 0;
      }
    }

    marker_pub.publish(marker);
  
    ros::spinOnce();
    r.sleep();
  }
}
