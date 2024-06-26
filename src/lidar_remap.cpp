#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include "sensor_msgs/PointCloud2.h"

class lidar_remap
{

    

private:
    ros::NodeHandle n; 
    std::string header;
    sensor_msgs::PointCloud2 messaggio;
    ros::Subscriber sub;
    ros::Publisher pub; 
	
public:
  	lidar_remap(){
  	sub = n.subscribe("/ugv/rslidar_points", 1, &lidar_remap::callback, this);
	pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_in", 1);	
}


void callback(const sensor_msgs::PointCloud2& msg){
    messaggio = msg;
    messaggio.header.frame_id = "base_laser_link";
    messaggio.header.stamp=ros::Time::now();
    pub.publish(messaggio);
}

};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "lidar_remap");
 	lidar_remap my_lidar_remap;
 	ros::spin();
 	return 0;
}
