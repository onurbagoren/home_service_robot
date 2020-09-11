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

        vector< vector<float> > goals{ { 1.0, 3.0 }, { 4.0, 0.0 } };

        void odomCallBack( const nav_msgs::Odometry::ConstPtr &msg );
        void goalCallBack( const geometry_msgs::Pose &msg );
        void publishMarker( void );
        bool robotAtMarker( geometry_msgs::Pose &robot_position, geometry_msgs::Pose &goal_position );
};

AddMarkers::AddMarkers()
{
    marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );

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

void AddMarkers::publishMarker( void )
{
    while( marker_pub.getNumSubscribers() < 1 )
    {
        ROS_WARN_ONCE("Create a subscriber to the marker");
        sleep(1);
    }
    
    for( int i = 0; i < goals.size(); i++ )
    {
        marker.pose.position.x = goals[i][0];
        marker.pose.position.y = goals[i][1];

        marker.action = visualization_msgs::Marker::ADD;

        ROS_INFO("Publising marker");
        marker_pub.publish(marker);
        ROS_INFO("Waiting 5 seconds");
        sleep(5);
        if( i != goals.size() - 1 )
        {
            ROS_INFO("Deleting marker");
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish( marker );
            ROS_INFO("Waiting 5 seconds");
            sleep(5);
        }
    }

}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "add_markers" );
    AddMarkers addmarker;
    ros::Rate r(20);
    ros::spin();
    return 0;
}