#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

#include <string>

class odom_to_tf{
public:
  	odom_to_tf(){

    // get the name from the static parameter
    child_param = ros::this_node::getName() + "/child_frame";
    root_param = ros::this_node::getName() + "/root_frame"; 
    
  	sub = n.subscribe("/input_odom", 1, &odom_to_tf::callback, this);
    
}


void callback(const nav_msgs::Odometry& msg){
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(msg.pose.pose.position.x,msg.pose.pose.position.y, msg.pose.pose.position.z));
  const tfScalar x = msg.pose.pose.orientation.x;
  const tfScalar y = msg.pose.pose.orientation.y;
  const tfScalar z = msg.pose.pose.orientation.z;
  const tfScalar w = msg.pose.pose.orientation.w;
  tf::Quaternion q(x,y,z,w);                 

  transform.setRotation(q);

  n.getParam(child_param, child_frame);
  n.getParam(root_param, root_frame);
  

  //ROS_INFO_STREAM( "child frame: "<<child_frame);
  //ROS_INFO_STREAM( "root frame: "<<root_frame);
  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
}

private:
  ros::NodeHandle n; 
  tf::TransformBroadcaster br;
  ros::Subscriber sub;

  // get the name from the static parameter
  std::string child_frame, root_frame, child_param, root_param;
  
};


int main(int argc, char **argv){
 ros::init(argc, argv, "gps_to_odom");
 
 odom_to_tf node;
 ros::spin();
 return 0;
}

