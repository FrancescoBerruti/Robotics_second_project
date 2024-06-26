#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>

#include "math.h"
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept> // std::runtime_error
#include <sstream>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

bool active = true;

std::vector<std::vector<double>> extract_goals(std::string path)
{   
    std::vector<std::vector<double>> goals;
    // Create an input filestream to read csv file
    std::ifstream myFile(path);
    // make sure file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");
    std::string line;
    double val;
    std::vector<double> ll;
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
                                
                ll.push_back(val);
                
                // If the next token is a comma, ignore it and move on
                if(ss.peek() == ',') ss.ignore();
                // ROS_INFO_STREAM(val);   
            }
            goals.push_back(ll);
            ll.clear();
        }
        return goals;
    }
    
}

geometry_msgs::Pose get_pose(const std::vector<std::vector<double>> &goals)
{
    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = goals[0][0];
    goal_pose.position.y = goals[0][1];

    goal_pose.orientation.w = cos(goals[0][2]/2);
    goal_pose.orientation.z = sin(goals[0][2]/2);   

    //ROS_INFO_STREAM("x: " << goal_pose.position.x << " y: " << goal_pose.position.y);

    return goal_pose;
}

void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    active=false;
}

void activeCb() {
    ROS_INFO("Goal just went active");
}

void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
//     ROS_INFO("Got Feedback at pose:");
//     ROS_INFO_STREAM(feedback->base_position.pose);
 }

void preemptTimerCallback(const ros::TimerEvent&, Client* client) {
    if (client->getState() == actionlib::SimpleClientGoalState::ACTIVE || client->getState() == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("Preempting the current goal due to timeout.");
        active=false;

        client->cancelGoal();
    }
}


int main(int argc, char **argv)
{
 	ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle n; 

    // create a vector of vectors to store the goal pose
    std::vector<std::vector<double>> goals;
    std::string path;
    std::string param_name;

    param_name= ros::this_node::getName() + "/waypointpath";
    n.getParam(param_name, path);
    goals = extract_goals(path);

    // the name of the action server should be move_base i think
    Client client("move_base", true);
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    move_base_msgs::MoveBaseGoal action_goal;

    // define duration for stopping the goal if it takes too long
    double duration = 60.0; 
    ros::Timer timer;
    ros::Rate loop_rate(1);

    while(goals.size() > 0){
        // variable to know if the goal is still active or not
        active=true;

        // setup the action goal and send it
        action_goal.target_pose.pose = get_pose(goals);
        action_goal.target_pose.header.frame_id="map";
        client.sendGoal(action_goal, &doneCb, &activeCb, &feedbackCb);

        // Setup a timer to preempt the goal after the specified duration
        timer = n.createTimer(ros::Duration(duration), boost::bind(preemptTimerCallback, _1, &client), true);

        // spin the node
        ros::Rate loop_rate(1);
        while(ros::ok() && active){
            // ROS_INFO("doing other processing");
            ros::spinOnce();
            loop_rate.sleep();
        }

        // the goal is reached/aborted, so delete the goal from the vector
        ROS_INFO("Assigning new goal");
        goals.erase(goals.begin()); 
        // and send the new goal (if there exists)
    }

 	return 0;
}









