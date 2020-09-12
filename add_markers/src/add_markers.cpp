#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
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
        ros::Publisher marker_pub;
        ros::Subscriber goal_sub;
        ros::Subscriber odom_sub;
        void odomCallBack( const nav_msgs::Odometry::ConstPtr &msg );
        void goalCallBack( const move_base_msgs::MoveBaseGoal &msg );
        vector< vector <float> > marker_locations{ { 1.0, 3.0, 1.0 }, { 4.0, 0.0, 1.0 } };

        visualization_msgs::Marker marker;
        geometry_msgs::Pose robot_pose_;
        geometry_msgs::Pose marker_pose_;
        bool start = true;
        bool pick_up = true;
        bool searching = true;

        bool closeToMarker( geometry_msgs::Pose robot_pose, geometry_msgs::Pose marker_pose );
};

AddMarkers::AddMarkers()
{
    marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
    goal_sub = n.subscribe( "/goal", 10, &AddMarkers::goalCallBack, this );
    odom_sub = n.subscribe( "/odom", 10, &AddMarkers::odomCallBack, this );

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    while( true )
    {
        if( closeToMarker( robot_pose_, marker_pose_ ) )
        {
            marker.pose = marker_pose_;
            cout << marker.pose << endl;    
            marker.action = visualization_msgs::Marker::ADD;
            if( start )
            {
                ROS_INFO("Picking up, wait 5 seconds.");
                marker.action = visualization_msgs::Marker::DELETE;
                start = !start;

            } else if( pick_up ) {
                sleep(5);
                marker_pub.publish( marker );
                pick_up = !pick_up;
            } else {
                ROS_INFO( "Drop off, appear object at (x, y, w) = (%f, %f, %f)", marker_locations[1][0], marker_locations[1][1], marker_locations[1][2] );
                marker_pub.publish(marker);
                sleep(10);
                break;
            }
        }
        ros::spinOnce();
    }
}

AddMarkers::~AddMarkers()
{

}

void AddMarkers::odomCallBack( const nav_msgs::Odometry::ConstPtr &msg )
{
    robot_pose_ = msg->pose.pose;
}

void AddMarkers::goalCallBack( const move_base_msgs::MoveBaseGoal &msg )
{
    ROS_INFO("Marker x: %f, y: %f, w: %f", msg.target_pose.pose.position.x, msg.target_pose.pose.position.y, msg.target_pose.pose.orientation.w);
    marker_pose_ = msg.target_pose.pose;
}

bool AddMarkers::closeToMarker( geometry_msgs::Pose robot_pose, geometry_msgs::Pose marker_pose )
{
    float x_diff = robot_pose.position.x - marker_pose.position.x;
    float y_diff = robot_pose.position.y - marker_pose.position.y;
    float position_distance = sqrt( pow(x_diff, 2) + pow(y_diff, 2) );
    if( position_distance <= 0.2 )
        return true;
    return false;
}

int main( int argc, char** argv )
{
    cout << "here" << endl;
    ros::init( argc, argv, "add_markers" );
    AddMarkers addmarker;
    return 0;
}