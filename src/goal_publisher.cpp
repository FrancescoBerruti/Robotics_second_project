#include <ros/ros.h>

#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <fstream>
#include <vector>
#include <string>
#include <stdexcept> // std::runtime_error
#include <sstream> 

class GoalPublisher
{

private:
    ros::NodeHandle n; 

    move_base_msgs::MoveBaseAction Client;
    // Setup a timer to preempt the goal after the specified duration
    ros::Timer timer;
    // define duration for stopping the goal if it takes too long
    double duration; 

    // create a vector of vectors to store the goal pose
    std::vector<std::vector<int>> goals;

    move_base_msgs::MoveBaseActionGoal action_goal;
    geometry_msgs::Pose goal_pose;
	
public:
    GoalPublisher(){

        // Create an input filestream to read csv file
        std::ifstream myFile("/waypoints.csv");
        // make sure file is open
        if(!myFile.is_open()) throw std::runtime_error("Could not open file");
        std::string line;
        int val;
        // Read data line by line
        if(myFile.good())
        {
            // Extract the first line in the file

            while(std::getline(myFile, line))
            {
                // Create a stringstream of the current line
                std::stringstream ss(line);
                
                // Extract each integer
                while(ss >> val){
                    
                    // Add the current integer to the 'colIdx' column's values vector
                    std::vector<int> ll;
                    ll.pushback(val);
                    
                    // If the next token is a comma, ignore it and move on
                    if(ss.peek() == ',') ss.ignore();
                    
                    // Increment the column index
                    colIdx++;
                }
                goals.pushback(ll);
            }
        }
        
        // the name of the server should be move_base i think
        Client client("move_base", true);
        ROS_INFO("Waiting for action server to start.");

        client.waitForServer();

        ROS_INFO("Action server started, sending goal.");
            
        // duration in seconds
        duration = 60.0; 
         
        client.sendGoal(action_goal, &GoalPublisher::doneCb, &GoalPublisher::activeCb, &GoalPublisher::feedbackCb);

        // Setup a timer to preempt the goal after the specified duration
        timer = n.createTimer(ros::Duration(duration), boost::bind(preemptTimerCallback, _1, &client), true);


    }

    void set_parameters(first_project::parametersConfig &config, uint32_t level) {
        /*ROS_INFO("Reconfigure Request: %d %s", 
                config.int_param,
                config.header.c_str());*/

        header = config.header.c_str();
    }

    void callback(const sensor_msgs::PointCloud2& msg){
        messaggio = msg;
        messaggio.header.frame_id = header;
        messaggio.header.stamp=ros::Time::now();
        pub.publish(messaggio);

    }

};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "goal_publisher");
 	GoalPublisher goal_publisher;
    ros::Rate loop_rate(1);

    while (ros::ok()){
	      ROS_INFO("doing other processing");
        ros::spinOnce();

        loop_rate.sleep();
    }

 	return 0;
}









