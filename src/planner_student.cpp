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


/*---------------------------------------------------------------------
Gives the shortest angle between 2 angles in the trigonometric circle
---------------------------------------------------------------------*/
float shortestAngleDiff(float th1, float th2)
{
	float anglediff;

	// Method 1
	anglediff = fmod( (th1 - th2) + M_PI, 2*M_PI) - M_PI ;
	

	// Method 2
	/*
	anglediff = fmod( (th1 - th2) , 2*M_PI);

	if( anglediff < 0.0 ) {
		if( fabs(anglediff) > (2*M_PI + anglediff) ) {
			anglediff = 2*M_PI + anglediff;
		}
	} else {
		if( anglediff > fabs(anglediff - 2*M_PI) ) {
			anglediff = anglediff - 2*M_PI;
		}
	}*/

	return anglediff;
}


/*---------------------------------------------------------------------
FUNC
Add a pose p (type Pose2D) in the Path to follow
Argument clear (type bool) clear the existing Path when set to true
---------------------------------------------------------------------*/
void addPose2DToPath(Pose2D p, bool clear )
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
	if(clear) pathPoses.clear();
	pathPoses.push_back(ps);

}


/*---------------------------------------------------------------------
SERVICE
Clear the previous Path 
Add the only one Pose received by the client to the Path 
---------------------------------------------------------------------*/
bool goalService(local_planner_student::localGoal::Request& request, local_planner_student::localGoal::Response& response)
{
	response.possible.data = isObstacle?false:true;

	if(!isObstacle) {
		addPose2DToPath(request.goalPose2D, true);
		ROS_INFO("Goal to x = %.2f    y = %.2f",request.goalPose2D.x, request.goalPose2D.y );
	}

	return (!isObstacle);
}

/*---------------------------------------------------------------------
SERVICE
Clear the previous Path 
Add the Path received by the client to the Path with a TF transform between the frame_id
---------------------------------------------------------------------*/
bool pathService(local_planner_student::Path::Request& request, local_planner_student::Path::Response& response)
{
	tf::TransformListener listener;
	tf::StampedTransform transform;


	try{
		ros::Time now = ros::Time(0);
		listener.waitForTransform(request.pathToGoal.header.frame_id, "/odom", now, ros::Duration(2.0));
		listener.lookupTransform(request.pathToGoal.header.frame_id, "/odom", now, transform);

		ROS_INFO("Tranform from %s to /odom is x = %.2f    y = %.2f",request.pathToGoal.header.frame_id.c_str(), transform.getOrigin().x(), transform.getOrigin().y() );
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		//ros::Duration(1.0).sleep();
	}

	for(int i=0; i<request.pathToGoal.poses.size(); i++) {
		request.pathToGoal.poses.at(i).pose.position.x ; // TODO : Apply here the transform in x
		request.pathToGoal.poses.at(i).pose.position.y ; // TODO : Apply here the transform in y
		
		ROS_INFO("# Pose %d : x = %.2f   y = %.2f", i, request.pathToGoal.poses.at(i).pose.position.x, request.pathToGoal.poses.at(i).pose.position.y );
	}

	//ROS_INFO("### New path with %d poses", (int)request.pathToGoal.poses.size() );	

	pathPoses.clear();
	pathPoses.reserve(request.pathToGoal.poses.size());
	copy(request.pathToGoal.poses.rbegin(),request.pathToGoal.poses.rend(),back_inserter(pathPoses));

	ROS_INFO("### New path with %d poses", (int)pathPoses.size() );	

	response.success.data = true;

	return true;
}

/*---------------------------------------------------------------------
CALLBACK
odometry callback
Set curPose2D with current robot x y theta
---------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------
CALLBACK
laser scan callback
Detect (set isObstacle = true) and log if obstacle
---------------------------------------------------------------------*/
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

		// Suggestion : 
		// - To pop your vector use if(pathPoses.size() > 1) pathPoses.pop_back();
		// - To use the current pose in Path use pathPoses.back().pose.


		velocity_pub.publish(twist);

		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
