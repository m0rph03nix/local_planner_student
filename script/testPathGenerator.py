#!/usr/bin/env python
# license removed for brevity

__author__ = 'Raphael LEBER'


import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from copy import deepcopy

from local_planner_student.srv import Path as PathToGoal 

class PathGenerator:

    def __init__(self):
        rospy.init_node('testPathGenerator')


    def sendTestPath_client(self):
        print "Wait for service" 
        rospy.wait_for_service('/move_to/pathGoal')
        print "Service found"
        try:
            print "Connect to service"
            pathToGoal = rospy.ServiceProxy('/move_to/pathGoal', PathToGoal)
            print "Service proxy created"
            fb = pathToGoal( self.generatePath() )
            print "Path sent to service"
            return fb.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e     


    def generatePath(self):
        pm = Path() 
        pm.header.frame_id = "/map"
        pm.header.stamp = rospy.Time.now()

        ps = PoseStamped()
        ps.header.frame_id = pm.header.frame_id
        ps.header.stamp = rospy.Time.now()

        # One way
        ps.pose.position.x = 2.8
        ps.pose.position.y = 2.8
        ps.pose.orientation.w = 1.0
        pm.poses.append( deepcopy(ps) )

        ps.pose.position.x = 4.0
        ps.pose.position.y = 3.0
        pm.poses.append( deepcopy(ps) )

        ps.pose.position.x = 8.0
        ps.pose.position.y = 6.0
        pm.poses.append( deepcopy(ps) )

        ps.pose.position.x = 6.0
        ps.pose.position.y = 7.0
        #pm.poses.append( deepcopy(ps) )

        # Way back
        ps.pose.position.x = 8.0
        ps.pose.position.y = 6.0
        #pm.poses.append( deepcopy(ps) )  

        ps.pose.position.x = 4.0
        ps.pose.position.y = 3.0
        pm.poses.append( deepcopy(ps) )   
        
        ps.pose.orientation.w = 1.0
        ps.pose.orientation.z = 0.0
        ps.pose.position.x = 2.8
        ps.pose.position.y = 2.8
        pm.poses.append( deepcopy(ps) )                   

        return pm                       


        


#******************************************************************************************
#***************************************   MAIN   *****************************************
#******************************************************************************************

if __name__ == '__main__':
    try:

        pg = PathGenerator()
        pg.sendTestPath_client()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
