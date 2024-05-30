#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include "sensor_msgs/PointCloud2.h"

class lidar_remap
{

    std::string header;
    sensor_msgs::PointCloud2 messaggio;

    private:
    ros::NodeHandle n; 

    ros::Subscriber sub;
    ros::Publisher pub; 
	
public:
  	lidar_remap(){
  	sub = n.subscribe("/ugv/rslidar_points", 1, &lidar_remap::callback, this);
	pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);	
}


void callback(const sensor_msgs::PointCloud2& msg){
    messaggio = msg;
    messaggio.header.frame_id = "odom";
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
