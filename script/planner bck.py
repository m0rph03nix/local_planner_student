#!/usr/bin/env python
# license removed for brevity

__author__ = 'Raphael LEBER'


import rospy
import time
import tf
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point,PoseStamped,PointStamped
from nav_msgs.msg import Path

import numpy as np
import matplotlib as ml
import matplotlib.pyplot as pl
import heapq
from Queue import Queue,LifoQueue

import math


class Planner:

    def __init__(self, resolution, shortPathMethod,isLocalPlanner,inflate_radius):
        #init params
        self.shortPathMethodeSelected=shortPathMethod
        self.RESOLUTION=resolution
        self.isLocalPlanner=isLocalPlanner
        self.inflate_radius=inflate_radius

        # init ros node
        rospy.init_node('ShortPathMng', anonymous=True)

        # ------------------#
        #--- Subscriber ----#
        #-------------------#

        # get the current map
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
        # get the current goal for navigation
        rospy.Subscriber("/clicked_point", PointStamped, self.askForGoalCallback)

        # ------------------#
        # --- Publisher ----#
        # ------------------#

        # marker to display algorithm
        self.pub_marker = rospy.Publisher('process_algo', MarkerArray, queue_size=10)
        # Send goal to navigation stack
        #FIXME Need to call local planner and send a path
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)

        self.isMapComputed=False
        self.tflistener = tf.TransformListener()

        # ------------------#
        # ---- Service -----#
        # ------------------#
        rospy.wait_for_service('/move_to/pathGoal')
        try:
            self.local_planner_service = rospy.ServiceProxy('/move_to/pathGoal', Path_Planner)
            # resp1 = add_two_ints(x, y)
        except rospy.ServiceException, e:
            print "Service call failed: %s to get /move_to/pathgoal service" % e





#******************************************************************************************
#************************************   MAP PROCESSING   **********************************
#******************************************************************************************

    def shortestAngleDiff(self, th1, th2):
        
        anglediff = fmod( (th1 - th2) + math.pi, 2*math.pi) - math.pi     
        return anglediff

# Add a pose p (type Pose2D) in the Path to follow
# Argument clear (type bool) clear the existing Path when set to true
    def addPose2DToPath(p, clear )

        ps = PoseStamped()
        q = tf.createQuaternionFromRPY(0.0, 0.0, p.theta)
        ps.header.frame_id = "odom"
        ps.header.stamp = ros.Time.now()
        ps.header.seq = 0
        ps.pose.position.x = p.x
        ps.pose.position.y = p.y
        ps.pose.orientation.x = q.x()
        ps.pose.orientation.y = q.y()
        ps.pose.orientation.z = q.z()
        ps.pose.orientation.w = q.w()
        if(clear) pathPoses.clear()
        pathPoses.push_back(ps)



    def initDefaultPath(void)

        addPose2DToPath( curPose2D, true )



# Clear the previous Path
#Add the Path received by the client to the Path with a TF transform between the frame_id

    def pathService(local_planner_raph.Path.Request request, local_planner_raph.Path.Response& response)

        listener = tf.TransformListener()
        transform = tf.StampedTransform()

        try:
            now = ros.Time(0)
            listener.waitForTransform(request.pathToGoal.header.frame_id, "/odom", now, ros.Duration(2.0))
            listener.lookupTransform(request.pathToGoal.header.frame_id, "/odom", now, transform)
            rospy.loginfo("Tranform from %s to /odom is x = %.2f    y = %.2f" % (request.pathToGoal.header.frame_id.c_str(), transform.getOrigin().x(), transform.getOrigin().y() ) )
        except Exception as err
            rospy.loginfo("%s",str(err))

        for i in  xrange(request.pathToGoal.poses.size())  :
            request.pathToGoal.poses.at(i).pose.position.x -= transform.getOrigin().x()
            request.pathToGoal.poses.at(i).pose.position.y -= transform.getOrigin().y()
            rospy.loginfo("# Pose %d : x = %.2f   y = %.2f" % (i, request.pathToGoal.poses.at(i).pose.position.x, request.pathToGoal.poses.at(i).pose.position.y) );
        
        pathPoses.clear()
        pathPoses.reserve( request.pathToGoal.poses.size() )
        copy( request.pathToGoal.poses.rbegin(), request.pathToGoal.poses.rend(), back_inserter(pathPoses) )

        rospy.loginfo("### New path with %d poses", % int(pathPoses.size()) )

        response.success.data = True

        return True



if __name__ == '__main__':
    try:

        # ------------------#
        # --- ROS PARAM ----#
        # ------------------#
        # FIXME Check private or global ros param
        # RESOLUTION = rospy.get_param('~SHORT_PATH_RESOLUTION', 8)
        RESOLUTION = rospy.get_param('~SHORT_PATH_RESOLUTION', 4)
        shortPathMethodeSelected = rospy.get_param('~SHORT_PATH_METHOD', 'ASTAR')
        isLocalPlanner = rospy.get_param('~LOCAL_PLANNER_USED', True)
        inflate_radius= rospy.get_param('~INFLATE_RADIUS', 0.3)


        nav=ShortPathMng(RESOLUTION,shortPathMethodeSelected,isLocalPlanner,inflate_radius)

        #keep python node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
