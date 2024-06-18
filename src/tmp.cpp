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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class GoalPublisher
{

private:
    ros::NodeHandle n; 

    // Setup a timer to preempt the goal after the specified duration
    ros::Timer timer;
    // define duration for stopping the goal if it takes too long
    double duration; 

    Client client;

    // create a vector of vectors to store the goal pose
    std::vector<std::vector<int>> goals;

    move_base_msgs::MoveBaseActionGoal action_goal;
    geometry_msgs::Pose goal_pose;
	
public:
    
    GoalPublisher(const Client clientin){
        // the name of the server should be move_base i think
        client = clientin;
        
        ROS_INFO("Waiting for action server to start.");

        client.waitForServer();

        ROS_INFO("Action server started, sending goal.");
            
        // duration in seconds
        duration = 60.0; 
    }

    void extract_goals(){
        // Create an input filestream to read csv file
        std::ifstream myFile("/waypoints.csv");
        // make sure file is open
        if(!myFile.is_open()) throw std::runtime_error("Could not open file");
        std::string line;
        int val;
        std::vector<int> ll;
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
                    
                    ll.push_back(val);
                    
                    // If the next token is a comma, ignore it and move on
                    if(ss.peek() == ',') ss.ignore();
                    
                }
                goals.push_back(ll);
            }
        }
    }

    void send_goal(){
        goal_pose.position.x = goals[0][0];
        goal_pose.position.y = goals[0][1];

        goal_pose.orientation.w = cos(goals[0][2]/2);
	    goal_pose.orientation.z = sin(goals[0][2]/2);

        action_goal.goal.target_pose.pose = goal_pose;

        client.sendGoal(action_goal, &GoalPublisher::doneCb, &GoalPublisher::activeCb, &GoalPublisher::feedbackCb);

        // Setup a timer to preempt the goal after the specified duration
        timer = n.createTimer(ros::Duration(duration), boost::bind(GoalPublisher::preemptTimerCallback, _1, &client), true);

    }

    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseActionResult & result) {
        //ROS_INFO("Finished in state [%s]", state.toString().c_str());

        // the goal is reached, so delete the goal from the vector
        goals.erase(goals.begin()); 
        // and send the new goal (if there is)
        if (goals.size() > 0){
            send_goal();
        }
        
    }

    void activeCb() {
        ROS_INFO("Goal just went active");
    }

    void feedbackCb(const move_base_msgs::MoveBaseActionFeedback & feedback) {
        ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
    }

    void preemptTimerCallback(const ros::TimerEvent&, Client* client) {
        if (client->getState() == actionlib::SimpleClientGoalState::ACTIVE || client->getState() == actionlib::SimpleClientGoalState::PENDING) {
            ROS_INFO("Preempting the current goal due to timeout.");
            
            // the goal is aborted, so delete the goal from the vector
            goals.erase(goals.begin()); 
            // and send the new goal (if there is)
            if (goals.size() > 0){
                send_goal();
            }

            client->cancelGoal();
        }
    }

};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "goal_publisher");

    Client client("move_base", true);

 	GoalPublisher goal_publisher(client);

    goal_publisher.extract_goals();
    goal_publisher.send_goal();
    ros::Rate loop_rate(1);

    while (ros::ok()){
	    ROS_INFO("doing other processing");
        ros::spinOnce();

        loop_rate.sleep();
    }

 	return 0;
}









