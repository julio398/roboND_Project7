//May 4th 2019.  Queretaro, Mexico.  Final project of the Robot Software Enginnering course at Udacity. By Julio Aguilar G.  julio398<at>hotmail<dot>com

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

//This a global variable that is being used to store a int number that represents different scenarios identifed by the pose_robot_callback function. 

int goal_status;

//This is a call_back function which is subscribed to amcl poses that are based on Map poses, it stores those poses into two local variables. It also works as a control function since it compares the pose.position.<x,y> values to the desired map locations to pick up and deliver items, once it found a coincidence it updates de goal_status global variable then some loops inside the main function can draw or erase the markers. 

void pose_robot_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& my_pose_amcl)
{
  double pose_linear_x = my_pose_amcl->pose.pose.position.x;
  double pose_linear_y = my_pose_amcl->pose.pose.position.y;
  
  //Uncoment the line bellow to see all amcl x and y poses while running the node. 
  //ROS_INFO("AMCL_Poses received - linear_x:%1.2f, linear_y:%1.2f", (float)pose_linear_x, (float)pose_linear_y);

  //For practical purposes there I let some clearance in the target poses of pickup and dropoff zone. 
  if ((pose_linear_x > 6.00 && pose_linear_x < 6.15) && (pose_linear_y > 4.14 && pose_linear_y < 4.29)) {
       goal_status = 1;
  }

  if ((pose_linear_x > 3.38 && pose_linear_x < 3.53) && (pose_linear_y > -5.55 && pose_linear_y < -5.40)) {
       goal_status = 2;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber amcl_robot_sub = n.subscribe("amcl_pose", 100, pose_robot_callback);
  ros::Rate loop_rate(10);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 6.1;
  marker.pose.position.y = 4.28;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  //marker.lifetime = ros::Duration();

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
  marker_pub.publish(marker);
  ROS_INFO("Hey, can you please pick up this item?");
  sleep(1);   
  
  //This is the main loop used to identify if the robot reached the first location to pick up item and second to deliver, in this order, otherwise if robot somehow reach second location nothing is going to happen since is mandatory to pick up item first. 
  while (ros::ok()){  
    if (goal_status == 1){
      marker.action = visualization_msgs::Marker::DELETE;
      sleep(3);
      marker_pub.publish(marker);
      ROS_INFO("Thank you, the item was picked up!");
      sleep(1);
      //Only when the item was picked up at the first location we can proceed to deliver in 2nd location. 
      while (ros::ok())
      {
        if (goal_status == 2){

          marker.pose.position.x = 3.42;
          marker.pose.position.y = -5.54;
          marker.action = visualization_msgs::Marker::ADD;
          sleep(3);
          marker_pub.publish(marker);
          ROS_INFO("The Item has been delivered at drop off zone! Thank you!");
          while (ros::ok()){
            sleep(1);  
          }
        }
      ros::spinOnce();
      }
    }
    ros::spinOnce();
  }
  ros::spin();  
  return 0;
}      
