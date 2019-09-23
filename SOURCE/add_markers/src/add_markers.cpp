#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  // We need to execute 3 steps: show, delete, show shape
  uint32_t step = 0;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "robotnd_udacity";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

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
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Cycle between different iterations
    switch (step)
    {
      case 0:{
        ROS_INFO("Publish the marker at the pickup zone");
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = -7.5;
        marker.pose.position.y = -7.0;
        marker.pose.position.z = 1.0;
        break;
      }
        
      case 1: {
        ROS_INFO("Hide the marker");
        marker.action = visualization_msgs::Marker::DELETE;
        break;
     }
        
      case 2:{
        ROS_INFO("Publish the marker at the drop off zone");
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 7.5;
        marker.pose.position.y = -7.0;
        marker.pose.position.z = 1.0;
        break;
      }
    }
    
    marker_pub.publish(marker);
    sleep(5);
  
    ++step;
    
    if(step > 2){
        return 0;
    }
    r.sleep();
  }
}
