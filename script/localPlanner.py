#!/usr/bin/env python
# license removed for brevity
__author__ = 'Raphael LEBER'


import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from nav_msgs.msg import OccupancyGrid
#from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist, Point, PoseStamped, PointStamped, Pose2D
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan


from std_msgs.msg import Bool
from math import fabs, sqrt, atan2, pi, fmod

from local_planner_raph.srv import localGoal 
from local_planner_raph.srv import PathToGoal
from copy import copy, deepcopy


class LocalPlanner:
    """
        A very simplistic local planner !!!

        Move from one waypoint to another. Stop if an obstacle is detected
        pathPoses attribute contains a list of waypoints

        Suscribe (inputs):
            - laser scan
            - odom

        Publish (output)
            - robot velocity

        Service:        
            pathPoses attribute can be filled in with: 
                - goalService for a single waypoint
                - pathService for a full path
    """

    pathPoses = []
    currentTarget = Pose2D()
    curPose2D = Pose2D()
    isObstacle = False

    def __init__(self, K_linear, K_angular, Sat_linear, Sat_angular, Obstacle_range, Angle_to_allow_linear, Waypoint_error, Destination_error, Angle_error):
        # init params
        self.K_linear               = K_linear              # Proportionnal coefficient for linear velocity
        self.K_angular              = K_angular             # Proportionnal coefficient for angular velocity
        self.Sat_linear             = Sat_linear            # Max linear velocity
        self.Sat_angular            = Sat_angular           # Max angular velocity
        self.Obstacle_range         = Obstacle_range        # Distance below which we consider an obstacle
        self.Angle_to_allow_linear  = Angle_to_allow_linear # Below this value : angular control only. Above this value : angular and linear control together
        self.Waypoint_error         = Waypoint_error        # Euclidian distance error to a waypoint allowing to move to a new waypoint
        self.Destination_error      = Destination_error     # Euclidian distance error to the final waypoint below which we consider the position reached 
        self.Angle_error            = Angle_error           # Angular error below which we consider the final orientation reached

        # init ros node
        rospy.init_node('Planner', anonymous=False)

        # ------------------#
        #--- Subscriber ----#
        #-------------------#
        # get odometry
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        # get laser scan
        rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        rospy.loginfo("Planner Subscribers initialized")

        # ------------------#
        # --- Publisher ----#
        # ------------------#
        # velocity command
        self.pub_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=100)
        rospy.loginfo("Planner publishers initialized")

        # ------------------#
        # ---- Service -----#
        # ------------------#
        self.opg_Srv = rospy.Service('goalService', localGoal, self.goalService)
        self.opg_Srv = rospy.Service('pathService', PathToGoal, self.pathService)
        rospy.loginfo("Planner services initialized")

#******************************************************************************************
#*********************************   SUSCRIBERS CALLBACK   ********************************
#******************************************************************************************

    def odomCallback(self, odom):
        """
            Odometry callback
            Set curPose2D with current robot x y theta
        """
        o = copy(odom.pose.pose.orientation)

        orientation_list = [o.x, o.y, o.z, o.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.curPose2D.x = odom.pose.pose.position.x
        self.curPose2D.y = odom.pose.pose.position.y
        self.curPose2D.theta = yaw  
        rospy.loginfo("X: %f \t Y: %f \t theta: %f"  % (self.curPose2D.x, self.curPose2D.y, self.curPose2D.theta ) )

                
    def scanCallback(self, scan):
        """
            laser scan callback
            Detect (set isObstacle = true) and log if obstacle
        """        
        #bestAngle = lessObstaclesAngle()


        self.logTempo = 0
        scan_size = (scan.angle_max - scan.angle_min) / scan.angle_increment

        self.isObstacle = False

        for i in xrange(int(scan_size)) :
            if( scan.ranges[i] < self.Obstacle_range ) :
                self.isObstacle = True
                if (self.logTempo < 100) :
                    self.logTempo += 1
                else:
                    rospy.loginfo("### OBSTACLE : Distance[%f]   Angle[%f]"  % (scan.ranges[i],  scan.angle_min + i * scan.angle_increment) )
                    logTempo = 0


#******************************************************************************************
#**************************************   SERVICES   **************************************
#******************************************************************************************

    def goalService(self, req):
        """
            ROS Service method 
            Add the pose in arg req to the path (self.pathPoses) if no obstacles are seen
            Return std_msgs/Bool at True
        """
        self.addPose2DToPath(req.goalPose2D, True)
        rospy.loginfo("Goal to x = %.2f    y = %.2f"  % (req.goalPose2D.x, req.goalPose2D.y) )
        fb = Bool()
        fb.data = not self.isObstacle
        return fb


    def pathService(self, req) :
        """
            # Clear the previous Path (self.pathPoses)
            # Try to add the Path received by the client to the Path (self.pathPoses) with a TF transform between the frame_id and /odom
            # Return std_msgs/Bool message with True if the "try" succeded, else (exception case) return False
        """           
        fb = Bool()
        listener = tf.TransformListener()


        try:
            now = rospy.Time(0)
            listener.waitForTransform(req.pathToGoal.header.frame_id, "/odom", now, rospy.Duration(2.0))
            (pos, quat) = listener.lookupTransform(req.pathToGoal.header.frame_id, "/odom", now)
            x = pos[0]
            y = pos[1]
            rospy.loginfo("Tranform from %s to /odom is x = %.2f    y = %.2f" % (req.pathToGoal.header.frame_id, x, y ) )
     
            for i in xrange( len(req.pathToGoal.poses) ) : 
                req.pathToGoal.poses[i].pose.position.x -= x
                req.pathToGoal.poses[i].pose.position.y -= y
                rospy.loginfo("# Pose %d : x = %.2f   y = %.2f" % (i, req.pathToGoal.poses[i].pose.position.x, req.pathToGoal.poses[i].pose.position.y) )

            del self.pathPoses[:]
            #self.pathPoses.insert(0,ps)
            self.pathPoses = deepcopy( req.pathToGoal.poses )

            rospy.loginfo("### New path with %d poses" % int( len(self.pathPoses)  ) )

            #response.success.data = True

            fb.data = True


        except Exception as err:
            rospy.loginfo("%s",str(err))
            #response.success.data = False
            fb.data = False
            
        return fb

#******************************************************************************************
#********************************   OBSTACLES COMPUTATION   *******************************
#******************************************************************************************

    def lessObstaclesAngle(self, scan):
        """
            #TODO
        """ 
        scan_size = (scan.angle_max - scan.angle_min) / scan.angle_increment

        nb_d_scan = 3
        d_scan_size = scan_size / nb_d_scan
        half_scan_size = scan_size / 2

        front_score = 0
        left_score = 0
        right_score = 0

        for i in xrange( d_scan_size, 2*d_scan_size) :
            if( scan.ranges[i] < self.Obstacle_range ) :
                front_score = front_score + 1

        for i in xrange( 0, d_scan_size) :
            if( scan.ranges[i] < self.Obstacle_range ) :
                left_score = left_score + 1  

        for i in xrange( 2*d_scan_size, scan_size) :
            if( scan.ranges[i] < self.Obstacle_range ) :
                right_score = right_score + 1   

        scores = [left_score, front_score, right_score]   

        index_min = min( xrange(len(scores)), key=scores.__getitem__)     

        if index_min == 0 :
            #left
            return scan.angle_increment * half_scan_size

        elif index_min == 1 :
            #straight
            return scan.angle_increment * half_scan_size

        elif index_min == 2 :
            #right
            return -scan.angle_increment * half_scan_size            


    def computeNewWaypoint(self, angle):
        angleCurTarget = atan2( self.pathPoses[0].position.y - self.curPose2D.y , self.pathPoses[0].position.x - self.curPose2D.x )
        angleCurTarget - angle


#******************************************************************************************
#**********************************   GOALS COMPUTATION   *********************************
#******************************************************************************************

    def shortestAngleDiff(self, th1, th2):
        """
            Returns the shortest angle between 2 angles in the trigonometric circle
        """        
        anglediff = fmod( (th1 - th2), 2*pi)

        if anglediff < 0.0:
            if fabs(anglediff) > (2*pi + anglediff) :
                anglediff = 2*pi + anglediff
        else:
            if anglediff > fabs(anglediff - 2*pi) :
                anglediff = anglediff - 2*pi

        return anglediff


    def addPose2DToPath(self, p, clear ) :
        """
            Add a pose p (type Pose2D) in the Path (self.pathPoses) to follow
            Argument clear (type bool) clear the existing Path (self.pathPoses) when set to true
        """   

        ps = PoseStamped()
        q = quaternion_from_euler(0.0, 0.0, p.theta)
        ps.header.frame_id = "odom"
        ps.header.stamp = rospy.Time.now()
        ps.header.seq = 0
        ps.pose.position.x = p.x
        ps.pose.position.y = p.y
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]   

        if(clear) : 
            del self.pathPoses[:]
        self.pathPoses.append(ps)


    def computeDistAngle(self):
        """
            Compute the euclidian distance to the target and the angle between robot orientation and its target
            Return a tupple (distance, angle) with distance and angle to the target
        """           

        if len(self.pathPoses) > 0 :
            pathPosePosition = self.pathPoses[0].pose.position
            distCurTarget = sqrt(pow(pathPosePosition.x - self.curPose2D.x, 2) + pow(pathPosePosition.y - self.curPose2D.y, 2) )
            angleCurTarget = atan2( pathPosePosition.y - self.curPose2D.y , pathPosePosition.x - self.curPose2D.x )
            angle = self.shortestAngleDiff(angleCurTarget, self.curPose2D.theta)

            return (distCurTarget, angle)
        
        else:
            return (0, 0)



    def computeFinalOrientation(self):
        """
            Compute the final orientation based on the last waypoint orientation 
            Return an angle (float)
        """
        if len(self.pathPoses) > 0 :
            o = self.pathPoses[0].pose.orientation
            (R, P, Y) = euler_from_quaternion((o.x, o.y, o.z, o.w))
            angle = self.shortestAngleDiff(Y, self.curPose2D.theta)
            return angle       
        else:
            return 0



    def pathSequencer(self, dist, angle, finalOrientation):
        """
            Switch between the following sequences : "New Goal" <--> "Reach in progress" --> "Last Goal position Reached" --> "Last Goal pose (position + orientation) Reached"
            :return: One of the string: "New Goal", "Reach in progress",  "Last Goal position Reached", "Last Goal pose (position + orientation) Reached"
        """
        if(dist < self.Waypoint_error ) and len(self.pathPoses) > 1 :
            del self.pathPoses[0]
            #computeVelocity
            rospy.loginfo("# New goal : x=%f ; y=%f"  % ( self.pathPoses[0].pose.position.x, self.pathPoses[0].pose.position.y ) )
            return "New Goal"

        elif(dist < self.Destination_error ) and len(self.pathPoses) == 1 :
            if fabs(finalOrientation) >= self.Angle_error :
                state = "Last Goal position Reached"
            else:
                state = "Last Goal pose (position + orientation) Reached"

            rospy.loginfo("# %s : X = %.2f ; Y = %.2f "  % ( state, self.pathPoses[0].pose.position.x, self.pathPoses[0].pose.position.y ) )
            return state

        return "Reach in progress"



    def computeVelocity(self, dist, angle, goalState) :
        """
            Compute linear and angular velocity
            :param dist: Euclidian distance to the waypoint
            :param angle: Angle to the waypoint
            :param goalState: string giving the state out the pathSequencer method
            :rtype: geometry_msgs.msg/Twist
        """

        twist = Twist()

        twist.angular.z = min([angle * self.K_angular, self.Sat_angular ] )

        if(fabs(angle) < self.Angle_to_allow_linear):
            if "New Goal" == goalState or "Reach in progress" == goalState :
                if self.isObstacle == True:
                    twist.linear.x = 0
                else:
                    twist.linear.x = min(dist * self.K_linear, self.Sat_linear)   

        if "Last Goal pose (position + orientation) Reached" == goalState:
            twist.angular.z = 0

        return twist   
        


    def localPlanning(self):
        """
            Main loop (20Hz):
                1/ Call method to compute distance and angle to waypoint
                2/ If last waypoint (goal), Call method to compute final orientation 
                3/ Call method to manage the sequence and get the goalState
                4/ Depending on the case:
                    a/ Call method to compute new distance and angle to waypoint
                    b/ set angle to final orientation
                5/ Call method to compute velocity
                6/ Publish velocity
        """
        rate = rospy.Rate(20) # 20hz

        while not rospy.is_shutdown():

            (dist, angle) = self.computeDistAngle()
            if len( self.pathPoses ) == 1 :
                finalOrientation = self.computeFinalOrientation()
            else:
                finalOrientation = 0

            goalState = self.pathSequencer(dist, angle, finalOrientation)
            if "New Goal" == goalState:
                (dist, angle) = self.computeDistAngle()
            elif "Last Goal position Reached" == goalState  or  "Last Goal pose (position + orientation) Reached" == goalState :           
                angle = finalOrientation


            twist = self.computeVelocity(dist, angle, goalState)
            self.pub_velocity.publish(twist)
            rate.sleep()



#******************************************************************************************
#***************************************   MAIN   *****************************************
#******************************************************************************************

if __name__ == '__main__':
    try:

        # ------------------#
        # --- ROS PARAM ----#
        # ------------------#
        K_LINEAR = rospy.get_param('~K_LINEAR', 1)
        K_ANGULAR = rospy.get_param('~K_ANGULAR', 4)
        SAT_LINEAR = rospy.get_param('~SAT_LINEAR', 2.0)
        SAT_ANGULAR = rospy.get_param('~SAT_ANGULAR', pi/2.0 )
        OBSTACLE_RANGE = rospy.get_param('~OBSTACLE_RANGE', 0.5)
        ANGLE_TO_ALLOW_LINEAR = rospy.get_param('~ANGLE_TO_ALLOW_LINEAR', 0.2)
        WAYPOINT_ERROR = rospy.get_param('~WAYPOINT_ERROR', 0.16)
        DESTINATION_ERROR = rospy.get_param('~DESTINATION_ERROR', 0.003)
        ANGLE_ERROR = rospy.get_param('~ANGLE_ERROR', 0.2)


        # ------------------#
        # Start LocalPlanner#
        # ------------------#        
        localPlanner = LocalPlanner(K_LINEAR, K_ANGULAR, SAT_LINEAR, SAT_ANGULAR, OBSTACLE_RANGE, ANGLE_TO_ALLOW_LINEAR, WAYPOINT_ERROR, DESTINATION_ERROR, ANGLE_ERROR)

        localPlanner.localPlanning()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
