#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <vector>

using namespace std;

class AddMarkers
{
    public:
        AddMarkers();
        ~AddMarkers();

    private:
        ros::NodeHandle n;
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
        // void odomCallBack( const nav_msgs::Odometry::ConstPtr &msg );
        // void goalCallBack( const geometry_msgs::Pose &msg );
        vector< vector <float> > marker_locations{ { 3.0, 1.0 }, { -1.0, -2.0 } };
        visualization_msgs::Marker marker;
        uint32_t shape_of_marker = visualization_msgs::Marker::CUBE;
        void show_markers();
};

AddMarkers::AddMarkers()
{
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape_of_marker;

    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    show_markers();

}

AddMarkers::~AddMarkers()
{

}

void AddMarkers::show_markers()
{
    while( marker_pub.getNumSubscribers() < 1 )
    {
        ROS_WARN_ONCE("Create a subscriber to the marker (from Rviz)");
        sleep(1);
    }

    for( int i = 0; i < marker_locations.size(); i++)
    {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = marker_locations[i][0];
        marker.pose.position.y = marker_locations[i][1];
        
        marker_pub.publish( marker );
        sleep(5);
        marker.action = visualization_msgs::Marker::DELETE;
    }
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "add_markers" );
    AddMarkers addmarker;
    return 0;
}