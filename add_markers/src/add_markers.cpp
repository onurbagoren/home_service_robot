#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <vector>

using namespace std;

class AddMarkers
{
    public:
        AddMarkers(){

            marker_pub = n.advertise<visualization_msgs::Marker>("/visual_marker", 1);
            goal_sub = n.subscribe("/goal", 1, &AddMarkers::goalCallBack, this);
            odom_sub = n.subscribe("/odom", 1, &AddMarkers::odomCallBack, this);

            uint32_t shape = visualization_msgs::Marker::CUBE;

            visualization_msgs::Marker marker;

            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();

            marker.ns = "basic_shape";
            marker.id = 0;
            marker.type = shape;

            marker.pose.position.z = 0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 0.0;

            marker.scale.x = 1.0f;
            marker.scale.y = 1.0f;
            marker.scale.z = 1.0f;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            iterate_goals( marker );
        }
    private:
        ros::NodeHandle n;
        ros::Publisher marker_pub;
        ros::Subscriber goal_sub;
        ros::Subscriber odom_sub;
        geometry_msgs::Pose goal;
        geometry_msgs::Pose robot_odom;
        void goalCallBack( const geometry_msgs::Pose &msg );
        void odomCallBack( const nav_msgs::Odometry::ConstPtr &msg );
        vector< vector< double > > goals{ {1.0, 3.0, 1.0}, {4.0, 0.0, 1.0} };
        void iterate_goals( visualization_msgs::Marker &marker );
};

void AddMarkers::iterate_goals( visualization_msgs::Marker &marker )
{
    if( marker_pub.getNumSubscribers() < 1 )
    {
        ROS_INFO( "No subscribers to the marker pub." );
    } 
    for( unsigned int i = 0; i < goals.size(); i++)
    {
        marker.pose.position.x = goals[i][0];
        marker.pose.position.y = goals[i][1];
        marker.pose.orientation.z = goals[i][2]; 
        float x_diff = robot_odom.position.x - goal.position.x;
        float y_diff = robot_odom.position.y - goal.position.y;
        float abs_distance = sqrt( x_diff * x_diff + y_diff * y_diff );
        
        if ( abs_distance < 0.20 )
        {
            marker_pub.publish(marker);
        } else {
            ros::spinOnce();
        }
    }

};

void AddMarkers::goalCallBack( const geometry_msgs::Pose &msg )
{
    //update the goal information
    goal.position.x = msg.position.x;
    goal.position.y = msg.position.y;
    goal.position.z = msg.position.z;

    goal.orientation.x = msg.orientation.x;
    goal.orientation.y = msg.orientation.y;
    goal.orientation.z = msg.orientation.z;
    goal.orientation.w = msg.orientation.w;

};

void AddMarkers::odomCallBack( const nav_msgs::Odometry::ConstPtr &msg)
{
    //update the current pose of the robot
    robot_odom.position.x = msg->pose.pose.position.x;
    robot_odom.position.y = msg->pose.pose.position.y;
    robot_odom.position.z = msg->pose.pose.position.z;

    robot_odom.orientation.x = msg->pose.pose.orientation.x;
    robot_odom.orientation.y = msg->pose.pose.orientation.y;
    robot_odom.orientation.z = msg->pose.pose.orientation.z;
    robot_odom.orientation.w = msg->pose.pose.orientation.w;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    AddMarkers add_markers;
    ros::Rate r(1);
    ros::spin();

    return 0;
}
