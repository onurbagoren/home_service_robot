#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>

using namespace std;

class AddMarkers
{
    public:
        AddMarkers();
        ~AddMarkers();

    private:
        geometry_msgs::Pose marker_position;
        geometry_msgs::Pose robot_pose;
        visualization_msgs::Marker marker;

        ros::NodeHandle n;
        ros::Publisher marker_pub;;
        ros::Subscriber odom_sub;
        ros::Subscriber goal_sub;
        void odomCallBack( const nav_msgs::Odometry::ConstPtr &msg );
        void goalCallBack( const geometry_msgs::Pose &msg );
        void publishMarker( void );
        bool robotAtMarker( geometry_msgs::Pose &robot_position, geometry_msgs::Pose &goal_position );

        float position_threshold = sqrt(2);
        float orientation_threshold_w = 0.1;

        bool searching = false;
};

AddMarkers::AddMarkers()
{
    marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
    goal_sub = n.subscribe("/goal", 1000, &AddMarkers::goalCallBack, this);
    ROS_INFO( "Marker position: %f, %f", marker_position.position.x, marker_position.position.y );
    odom_sub = n.subscribe("/odom", 1000, &AddMarkers::odomCallBack, this);
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    publishMarker();

}

AddMarkers::~AddMarkers()
{

}

void AddMarkers::odomCallBack( const nav_msgs::Odometry::ConstPtr &msg )
{
    robot_pose.position.x = msg->pose.pose.position.x;
    robot_pose.position.y = msg->pose.pose.position.y;
    robot_pose.position.y = msg->pose.pose.position.y;
    robot_pose.orientation.x = msg->pose.pose.orientation.x;
    robot_pose.orientation.y = msg->pose.pose.orientation.y;
    robot_pose.orientation.z = msg->pose.pose.orientation.z;
    robot_pose.orientation.w = msg->pose.pose.orientation.w;
}

void AddMarkers::goalCallBack( const geometry_msgs::Pose &msg )
{
    ROS_INFO( "In goal call back" );
    marker_position.position.x = msg.position.x;
    marker_position.position.y = msg.position.y;
}

void AddMarkers::publishMarker( void )
{
    while( marker_pub.getNumSubscribers() < 1 )
    {
        ROS_WARN_ONCE("Create a subscriber to the marker");
        sleep(1);
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = marker_position.position;
    marker_pub.publish(marker);
    sleep(5);

    // if(!searching)
    // {
    //     ROS_INFO("Searching");
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = marker_position.position.x;
    //     marker.pose.position.y = marker_position.position.y;
    //     marker_pub.publish(marker);
    //     searching = true;
    // }
    // cout << "Marker_pose: " << marker.pose << endl;
    // if( !robotAtMarker( robot_pose, marker.pose ) )
    // {
    //     ROS_INFO("Not at marker");
    // }
    // else
    // {  
    //     ROS_INFO("At marker");
    //     searching = false;
    //     marker.action = visualization_msgs::Marker::DELETE;
    //     marker_pub.publish(marker);
    //     sleep(5);
    // }

}

bool AddMarkers::robotAtMarker( geometry_msgs::Pose &robot_position, geometry_msgs::Pose &goal_position )
{
    cout << robot_position.position.x << goal_position.position.x << endl;
    float x_diff = robot_position.position.x - goal_position.position.x;
    float y_diff = robot_position.position.y - goal_position.position.y;
    ROS_INFO( "%f, %f", x_diff, y_diff );
    float position_diff = sqrt( pow( x_diff, 2 ) + pow( y_diff, 2 ) );

    if( position_diff < position_threshold )
        return true;
    
    return false;
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "add_markers" );
    AddMarkers addmarker;
    ros::Rate r(20);
    ros::spin();
    return 0;
}