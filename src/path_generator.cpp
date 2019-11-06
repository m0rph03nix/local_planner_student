#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "local_planner_raph/localGoal.h"
#include "local_planner_raph/PathToGoal.h"
#include <tf/transform_datatypes.h>

#include <sstream>
#include <iterator>     // std::back_inserter
#include <vector>       // std::vector
#include <algorithm>    // std::copy

using std::vector;
using std::string;
using std::copy;
using std::back_inserter;
using geometry_msgs::Pose2D;

Pose2D iniPose2D;
Pose2D curPose2D;
Pose2D goalPose2D;
bool isObstacle = false;
double zAngSpeed = 0;

/* TODO :
 - finalisation de la méthode actuelle en ajoutant l'orientation finale
 - nouvelle méthode avec des splines (polynomes 3 ou 5 ??)
 - service avec waypoints de type nav_msgs/Path.msg
 - option : Création de paramètre comme la vitesse lin et ang
 - option : diminution de la vitesse à l'approche d'un obstacle et emission d'un service de blocage après un certain temps


*/


nav_msgs::Path generatePath(void)
{
    nav_msgs::Path pm;
    pm.header.frame_id = "/map";
    unsigned int id = 0;

	//vector<float> presetX;



	

    //for (int i=0; i < (int)presetX.size(); i++)
    //{
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = pm.header.frame_id;

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 3.0;
        ps.pose.position.y = 2.5;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 2.5;
        ps.pose.position.y = 2.0;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 2.5;
        ps.pose.position.y = 2.0;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 2.5;
        ps.pose.position.y = 2.5;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 2.0;
        ps.pose.position.y = 3.0;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);
    //}

    return pm;
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "path_generator");

	ros::NodeHandle n;

  	ros::ServiceClient client = n.serviceClient<local_planner_raph::PathToGoal>("/move_to/pathGoal");

	ros::Rate loop_rate(10);


	

	local_planner_raph::PathToGoal srv;

	srv.request.pathToGoal = generatePath();

	if (client.call(srv))
	{
		//srv.response.success ? ROS_INFO("Success") : ROS_INFO("Fail");
		 ROS_INFO("Path service");
	}
	else
	{
		ROS_ERROR("Failed to call the Path service");
		return 1;
	}

/*
	while (ros::ok())
	{

	}
*/

  return 0;
}










/* Dans odom 

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 0.0;
        ps.pose.position.y = 0.0;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 0.5;
        ps.pose.position.y = 0.0;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 0.5;
        ps.pose.position.y = 0.0;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 0.5;
        ps.pose.position.y = 0.5;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = 0.0;
        ps.pose.position.y = 0.5;
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);

*/
