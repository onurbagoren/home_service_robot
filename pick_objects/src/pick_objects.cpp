#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <chrono>
#include <thread>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

int main(int argc, char** argv)
{

    vector<vector< double > > goals{ {1.0, 3.0, 1.0}, {4.0, 0.0, 1.0}  };

    ros::init(argc, argv, "pick_objects");

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    

    for( int i = 0; i < goals.size(); i++)
    {

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goals[i][0];
        goal.target_pose.pose.position.y = goals[i][1];
        goal.target_pose.pose.orientation.w = goals[i][2];

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal reached");
        }else{
            ROS_INFO("Goal was not reached");
        }

        ROS_INFO("Waiting for 5 seconds");
        chrono::seconds dura(5);
        this_thread::sleep_for(dura);

    }

    ROS_INFO("Final goal reached");

    return 0;

}

