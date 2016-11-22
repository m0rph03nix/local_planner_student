#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "local_planner_student/localGoal.h"
#include "local_planner_student/Path.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <sstream>
#include <iterator>     // std::back_inserter
#include <vector>       // std::vector
#include <algorithm>    // std::copy

using std::vector;
using std::string;
using std::copy;
using std::back_inserter;
using geometry_msgs::Pose2D;
using geometry_msgs::PoseStamped;

Pose2D curPose2D;

bool isObstacle = false; // set true if obstaclke detected in scanCallback

const float Kp = 1; // linear speed coefficient
const float Ka = 4; // angular speed coefficient
const float linSat = 2.0; // linear speed saturation
const float angSat = M_PI / 2.0; // angular speed saturation

vector<PoseStamped> pathPoses;
vector<PoseStamped>::reverse_iterator currentTarget;



float shortestAngleDiff(float th1, float th2)
{
	float anglediff;

	// TODO :
	// - return shortest angle to turn to point to reach

	return anglediff;
}



void addPose2DToPath(Pose2D p )
{
	geometry_msgs::PoseStamped ps;
	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, p.theta);
    ps.header.frame_id = "odom";
    ps.header.stamp = ros::Time::now();
    ps.header.seq = 0;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
	ps.pose.orientation.x = q.x();
	ps.pose.orientation.y = q.y();
	ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
	pathPoses.clear();
    pathPoses.push_back(ps);

}


bool goalService(local_planner_student::localGoal::Request& request, local_planner_student::localGoal::Response& response)
{
	// TODO :
	// - clear pathPoses
	// - if no obstacle (isObstacle = false), create a new path (pathPoses) with just one Pose
		
	return true;
}

bool pathService(local_planner_student::Path::Request& request, local_planner_student::Path::Response& response)
{
	// TODO :
	// - Get transform frame_id to "/odom"
	// - clear pathPoses
	// - Copy request path to pathPoses with transform difference

	response.success.data = true;

	return true;
}


void odomCallback(const nav_msgs::Odometry odom)
{
	tf::Quaternion q(	odom.pose.pose.orientation.x, 
						odom.pose.pose.orientation.y, 
						odom.pose.pose.orientation.z, 
						odom.pose.pose.orientation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	// TODO :
	// - curPose2D.x = 
	// - curPose2D.y = 
	// - curPose2D.theta =

}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	short scan_size = ((scan->angle_max - scan->angle_min) / scan->angle_increment);
	int i=0;
	static int logTempo = 0;
	//ROS_INFO("### scan->angle_min: %f",  scan->angle_min);
	//ROS_INFO("### scan->angle_max: %f",  scan->angle_max);
	//ROS_INFO("### scan->angle_increment: %f",  scan->angle_increment);

	// TODO :
	// - if range among ranges < 0.25, 
	//   	then isObstacle = true;
	//		then log obstacle

}


int main(int argc, char **argv)
{
	pathPoses.clear();
	pathPoses.reserve(0);

	ros::init(argc, argv, "planner1");

	ros::NodeHandle n;

	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 100);
	ros::Subscriber sub1 = n.subscribe("odom", 1, odomCallback);
	ros::Subscriber sub2 = n.subscribe("scan", 1, scanCallback);
	
	ros::ServiceServer service_goal = n.advertiseService("/move_to/singleGoal", goalService);
	ros::ServiceServer service_path = n.advertiseService("/move_to/pathGoal", pathService);

	ros::Rate loop_rate(30);
	
	// TODO : init Path

	while (ros::ok())
	{
		geometry_msgs::Twist twist;
		float angle = 0;
		float distCurTarget = 0; // Distance to current point to reach
		float angleCurTarget = 0; // Angle to current point to reach

		// TODO :
		// - for each pose in pathPoses, turn until angle error > 0.02rad, then continue to turn and move forward in the same time until reaching point (with 0.02m error)
		// - use function shortestAngleDiff
		// Note: Current robot pose is in curPose2D


		velocity_pub.publish(twist);

		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
