#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

#include <string>
  

class odom_to_tf{
public:
  	odom_to_tf(){
    
  	sub = n.subscribe("/ugv/odom", 1, &odom_to_tf::callback, this);
   
}


void callback(const nav_msgs::Odometry& msg){
  tf::Transform transform;  
  std::string root_frame;
  std::string child_frame;
  std::string child_param, root_param;
  child_param = ros::this_node::getName() + "/child_frame";
  root_param = ros::this_node::getName() + "/root_frame";
  n.getParam(child_param, child_frame);
  n.getParam(root_param, root_frame); 

  transform.setOrigin(tf::Vector3(msg.pose.pose.position.x,msg.pose.pose.position.y, msg.pose.pose.position.z));
  const tfScalar x = msg.pose.pose.orientation.x;
  const tfScalar y = msg.pose.pose.orientation.y;
  const tfScalar z = msg.pose.pose.orientation.z;
  const tfScalar w = msg.pose.pose.orientation.w;
  tf::Quaternion q(x,y,z,w);                 

  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
}

private:
  ros::NodeHandle n; 
  tf::TransformBroadcaster br;
  ros::Subscriber sub;
};


int main(int argc, char **argv){
 ros::init(argc, argv, "odom_to_tf");
 
 odom_to_tf node;
 ros::spin();
 return 0;
}

