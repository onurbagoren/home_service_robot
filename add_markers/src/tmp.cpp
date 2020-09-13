#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>

using namespace std;

int main( int argc, char** argv )
{
    ros::init( argc, argv, "tmp");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    vector< vector< float > > goals{{-1.0, 0.0}, {1.0, 1.0}};

    uint32_t shape = visualization_msgs::Marker::CUBE;

    while( ros::ok() )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = shape;

        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration();

        while( marker_pub.getNumSubscribers() < 1 )
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
        }

        for( int i = 0; i < goals.size(); i++) 
        {
            marker.pose.position.x = goals[i][0];
            marker.pose.position.y = goals[i][1];
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish( marker );
            sleep(5);
            if( i == 0 ){
                marker.action = visualization_msgs::Marker::DELETE;
                marker_pub.publish( marker );
                sleep(5);
            }
        }

        break; 
        // r.sleep();
    }
    return 0;
}