#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

float pickupzone[2]={-3,5};
float dropoffzone[2]={4.5,7};
float pose_tollerence = 0.18;

bool PICK_UP_ZONE, DROP_OFF_ZONE = false; // Is robot at pick up zone or drop off zone?
bool PICKED_UP, DROPED_OFF = false; // Is marker picked up or dropped off?

ros::Publisher marker_pub;
ros::Subscriber odom_sub;
visualization_msgs::Marker marker;

int Publish_marker(visualization_msgs::Marker MARKER)
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
  marker_pub.publish(MARKER);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  float pos_x= msg->pose.pose.position.x;
  float pos_y= msg->pose.pose.position.y;

  PICK_UP_ZONE = sqrt(pow(pos_x - pickupzone[0], 2) + pow(pos_y - pickupzone[1], 2)) < pose_tollerence ? true : false;
  DROP_OFF_ZONE = sqrt(pow(pos_x - dropoffzone[0], 2) + pow(pos_y - dropoffzone[1], 2)) < pose_tollerence ? true : false;

  if (PICK_UP_ZONE && !PICKED_UP) 
  {
    ROS_INFO("Marker is picked up.\n");
    marker.color.a = 0;
    Publish_marker(marker);

    PICKED_UP = true;
  }
    
  else if (DROP_OFF_ZONE && !DROPED_OFF) 
  {
    ROS_INFO("Marker is droped off.\n");
    marker.color.a = 1;
    marker.pose.position.x = dropoffzone[0];
    marker.pose.position.y = dropoffzone[1];
    Publish_marker(marker);

    DROPED_OFF = true;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  odom_sub = n.subscribe("odom", 10, odom_callback);
  
  // Set up marker
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = -3;
  marker.pose.position.y = 5;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration();

  // Initially publish marker at pick up zone
  Publish_marker(marker);
    
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

