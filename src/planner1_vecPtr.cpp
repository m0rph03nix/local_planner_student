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
#include "local_planner_raph/localGoal.h"
#include "local_planner_raph/Path.h"
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

bool isObstacle = false;
double zAngSpeed = 0;
bool newTarget = true;
bool odomReady = false;
bool goalReached = false;

const float Kp = 1;
const float Ka = 4;
const float linSat = 2.0;
const float angSat = M_PI / 2.0;

vector<PoseStamped> pathPoses;
vector<PoseStamped>::reverse_iterator currentTarget;



/* TODO :
 - finalisation de la méthode actuelle en ajoutant l'orientation finale
 - nouvelle méthode avec des splines (polynomes 3 ou 5 ??)
 - service avec waypoints de type nav_msgs/Path.msg
 - option : Création de paramètre comme la vitesse lin et ang
 - option : diminution de la vitesse à l'approche d'un obstacle et emission d'un service de blocage après un certain temps


*/

float shortestAngleDiff(float th1, float th2)
{
	float anglediff = fmod( (th1 - th2) + M_PI, 2*M_PI) - M_PI ;


	if( anglediff < 0.0 ) {
		if( fabs(anglediff) > (2*M_PI + anglediff) ) {
			anglediff = 2*M_PI + anglediff;
		}
	} else {
		if( anglediff > fabs(anglediff - 2*M_PI) ) {
			anglediff = anglediff - 2*M_PI;
		}
	}

	return anglediff;
}



void addPose2DToPath(Pose2D p )
{
	geometry_msgs::PoseStamped ps;
	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, p.theta);
    ps.header.frame_id = "map";
    ps.header.stamp = ros::Time::now();
    ps.header.seq = 0;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
	ps.pose.orientation.x = q.x();
	ps.pose.orientation.y = q.y();
	ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
	ROS_INFO("### SIZE addtoPose 1  size=%d", (int)pathPoses.size());
	pathPoses.clear();
	ROS_INFO("### SIZE addtoPose 2  size=%d", (int)pathPoses.size());
	//pathPoses.reserve
    pathPoses.push_back(ps);
	pathPoses.push_back(ps);
	ROS_INFO("### SIZE addtoPose 3  size=%d", (int)pathPoses.size());
}


void initDefaultPath(void)
{
	addPose2DToPath( curPose2D );
}

/*
void initDefaultPath(void)
{
	geometry_msgs::PoseStamped ps;
	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, curPose2D.theta);
    ps.header.frame_id = "map";
    ps.header.stamp = ros::Time::now();
    ps.header.seq = 0;
    ps.pose.position.x = curPose2D.x;
    ps.pose.position.y = curPose2D.y;
	ps.pose.orientation.x = q.x();
	ps.pose.orientation.y = q.y();
	ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
	pathPoses.clear();
    pathPoses.push_back(ps);
	pathPoses.push_back(ps);
}*/


//bool goalCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
bool goalService(local_planner_raph::localGoal::Request& request, local_planner_raph::localGoal::Response& response)
{
	response.possible.data = isObstacle?false:true;

	if(!isObstacle) {
		//goalPose2D.x = request.goalPose2D.x;
		//goalPose2D.y = request.goalPose2D.y;
		//goalPose2D.theta = atan2(goalPose2D.y - curPose2D.y , goalPose2D.x - curPose2D.x);//request.goalPose2D.theta;
		addPose2DToPath(request.goalPose2D);
		currentTarget = pathPoses.rbegin();
		ROS_INFO("Goal to x = %.2f    y = %.2f",request.goalPose2D.x, request.goalPose2D.y );
		goalReached = false;
 	}
		
	return true;
}

bool pathService(local_planner_raph::Path::Request& request, local_planner_raph::Path::Response& response)
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
		request.pathToGoal.poses.at(i).pose.position.x -= transform.getOrigin().x();
		request.pathToGoal.poses.at(i).pose.position.y -= transform.getOrigin().y();
		ROS_INFO("# Pose %d : x = %.2f   y = %.2f", i, request.pathToGoal.poses.at(i).pose.position.x, request.pathToGoal.poses.at(i).pose.position.y );
	}

	//ROS_INFO("### New path with %d poses", (int)request.pathToGoal.poses.size() );	

	pathPoses.clear();
    pathPoses.reserve(request.pathToGoal.poses.size());
    copy(request.pathToGoal.poses.rbegin(),request.pathToGoal.poses.rend(),back_inserter(pathPoses));

	ROS_INFO("### New path with %d poses", (int)pathPoses.size() );	

	currentTarget = pathPoses.rbegin();

	goalReached = false;

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


	curPose2D.x = odom.pose.pose.position.x;
	curPose2D.y = odom.pose.pose.position.y;
	curPose2D.theta = yaw;

	zAngSpeed = odom.twist.twist.angular.z;

	odomReady = true;
/*
  	ROS_INFO("odom: [x = %f] [y = %f] [angle = %f]", 	curPose2D.x,
														curPose2D.y, 
														curPose2D.theta);
*/
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	short scan_size = ((scan->angle_max - scan->angle_min) / scan->angle_increment);
	int i=0;
	static int logTempo = 0;
	//ROS_INFO("### scan->angle_min: %f",  scan->angle_min);
	//ROS_INFO("### scan->angle_max: %f",  scan->angle_max);
	//ROS_INFO("### scan->angle_increment: %f",  scan->angle_increment);

	isObstacle = false;

	for(i=0; i<scan_size; i++)
	{
		if(scan->ranges.at(i) < 0.25) 
		{	
			isObstacle = true;
			if(logTempo < 100 ) logTempo++;
			else {
				ROS_INFO("### OBSTACLE : Distance[%f]   Angle[%f]",  scan->ranges.at(i),  scan->angle_min+(float)i*scan->angle_increment);
				logTempo = 0;
			}
		}
	}
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
	
	initDefaultPath();

	currentTarget = pathPoses.rbegin();

	while (ros::ok())
	{
		geometry_msgs::Twist twist;
		float distance = 0;
		float angleGoal = 0;
		float angle = 0;

		float distCurTarget = 0;
		float angleCurTarget = 0;

		//ROS_INFO("#1");

		distCurTarget = sqrt(pow(currentTarget->pose.position.x - curPose2D.x, 2) + pow(currentTarget->pose.position.y - curPose2D.y, 2) );

		if( distCurTarget < 0.02 ) 
		{
			if(currentTarget != pathPoses.rend()) {
				ROS_INFO("### CURRENT Pose  --- x=%.2f   y=%.2f", currentTarget->pose.position.x, currentTarget->pose.position.y);
				currentTarget++;
				newTarget = true;
				ROS_INFO("### NEXT Pose  --- x=%.2f   y=%.2f", currentTarget->pose.position.x, currentTarget->pose.position.y);
				ROS_INFO("### SIZE  --- x=%.2f   y=%.2f,    size=%d", pathPoses.rend()->pose.position.x, pathPoses.rend()->pose.position.y, (int)pathPoses.size());
				
			} else {
				if(angle >= 0.01) newTarget = true;
				if( (distCurTarget < 0.003) && (angle < 0.01) ) goalReached = true;
				ROS_INFO("### KEEP Pose  --- x=%.2f   y=%.2f", currentTarget->pose.position.x, currentTarget->pose.position.y);
			}
		}

		distCurTarget = sqrt(pow(currentTarget->pose.position.x - curPose2D.x, 2) + pow(currentTarget->pose.position.y - curPose2D.y, 2) );

		angleCurTarget = atan2( currentTarget->pose.position.y - curPose2D.y , currentTarget->pose.position.x - curPose2D.x );


		angleGoal = angleCurTarget;


		angle = shortestAngleDiff(angleGoal, curPose2D.theta);


		ROS_INFO("### Pose2d: X=[%.1f] %.1f    Y=[%.1f] %.1f    theta=[%.1f] %.1f  ---  dist=%.2f  angle=%.2f", currentTarget->pose.position.x, 
																												curPose2D.x, 
																												currentTarget->pose.position.y, 
																												curPose2D.y, 
																												angleGoal, 
																												curPose2D.theta, 
																												distCurTarget, angle);


		if(!goalReached) twist.angular.z = std::min(angle * Ka, angSat);

		if(newTarget) {
			if(angle < 0.01) {
				newTarget = false;
			}
		} else {
			if(!goalReached) twist.linear.x = std::min(distCurTarget * Kp, linSat);
		}
		
		
		

		velocity_pub.publish(twist);

		ros::spinOnce();

		loop_rate.sleep();

	}


  return 0;
}
