#!/usr/bin/env python3

__author__ = 'Raphael LEBER'

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose2D
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from math import fabs, sqrt, atan2, pi, fmod

from copy import deepcopy

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from local_planner_srvs.srv import LocalGoal
from local_planner_srvs.srv import PathToGoal

class LocalPlanner(Node):
    """
        A very simplistic local planner !!!

        Move from one waypoint to another. Stop if an obstacle is detected
        pathPoses attribute contains a list of waypoints

        Subscribe (inputs):
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

    def __init__(self):
        super().__init__('local_planner_node')
        # init params
       

        # Retrieve parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('K_LINEAR', 1.0),
                ('K_ANGULAR', 4.0),
                ('SAT_LINEAR', 2.0),
                ('SAT_ANGULAR', (3.14159265359 / 2.0)),
                ('OBSTACLE_RANGE', 0.),
                ('ANGLE_TO_ALLOW_LINEAR', 0.2),
                ('WAYPOINT_EPS', 0.16),
                ('DESTINATION_EPS', 0.003),
                ('ANGLE_EPS', 0.2)
            ]
        )

        # Proportionnal coefficient for linear velocity
        self.K_linear = self.get_parameter('K_LINEAR').get_parameter_value().double_value or 1.0
        # Proportionnal coefficient for angular velocity
        self.K_angular = self.get_parameter('K_ANGULAR').get_parameter_value().double_value or 4.0
        # Max linear velocity
        self.Sat_linear = self.get_parameter('SAT_LINEAR').get_parameter_value().double_value or 2.0
        # Max angular velocity
        self.Sat_angular = self.get_parameter('SAT_ANGULAR').get_parameter_value().double_value or (3.14159265359 / 2.0)  # Approximation of pi
        # Distance below which we consider an obstacle
        self.Obstacle_range = self.get_parameter('OBSTACLE_RANGE').get_parameter_value().double_value or 0.5
        # Above this value: angular control only. Below this value: angular and linear control together
        self.Angle_to_allow_linear = self.get_parameter('ANGLE_TO_ALLOW_LINEAR').get_parameter_value().double_value or 0.2
        # Euclidian distance error to a waypoint allowing to move to a new waypoint
        self.Waypoint_eps = self.get_parameter('WAYPOINT_EPS').get_parameter_value().double_value or 0.16
        # Euclidian distance error to the final waypoint below which we consider the position reached
        self.Destination_eps = self.get_parameter('DESTINATION_EPS').get_parameter_value().double_value or 0.003
        # Angular error below which we consider the final orientation reached
        self.Angle_eps = self.get_parameter('ANGLE_EPS').get_parameter_value().double_value or 0.2
       

        # tf2_ros transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------#
        #--- Subscribers ---#
        #-------------------#
        # get odometry
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        # get laser scan
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.isObstacle = False

        # ------------------#
        # --- Publisher ---#
        # ------------------#
        # velocity command
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ------------------#
        # ---- Services ----#
        # ------------------#
        self.goal_service = self.create_service(LocalGoal, 'goalService', self.goal_service_callback)
        self.path_service = self.create_service(PathToGoal, 'pathService', self.path_service_callback)

        # LOCAL PLANNING
        timer_period = 0.05  # 20Hz
        self.create_timer(timer_period, self.local_planning)

    #******************************************************************************************
    #*********************************   SUBSCRIBERS CALLBACK   ********************************
    #******************************************************************************************

    def odom_callback(self, odom):
        """
            Odometry callback
            Set curPose2D with current robot x y theta
        """
        
        #TODO for students : Fill in self.curPose2D (help : use euler_from_quaternion)


    def scan_callback(self, scan):
        """
            Laser scan callback
            Detect (set isObstacle = true) and log if obstacle
        """        
        
        #TODO for students : If an obstacle below self.Obstacle_range if detected, then self.isObstacle = True, False otherwise



    #******************************************************************************************
    #**************************************   SERVICES   **************************************
    #******************************************************************************************

    def goal_service_callback(self, request, response):
        """
            ROS Service method 
            Add the pose in arg req to the path (self.pathPoses) if no obstacles are seen
            Return std_msgs/Bool at True
        """
        self.add_pose_2d_to_path(request.goal_pose2d, True)
        self.get_logger().info("Goal to x = %.2f    y = %.2f"  % (request.goal_pose2d.x, request.goal_pose2d.y))
        response.possible.data = not self.isObstacle
        return response

    def path_service_callback(self, request, response):
        """
            Clear the previous Path (self.pathPoses)
            Try to add the Path received by the client to the Path (self.pathPoses) with a TF transform between the frame_id and /odom
            Return std_msgs/Bool message with True if the "try" succeeded, else (exception case) return False
        """           
        self.pathPoses.clear()

        try:
            transform = self.tf_buffer.lookup_transform("odom", request.path_to_goal.header.frame_id, self.get_clock().now(), rclpy.time.Duration(seconds = 10))

            for pose_stamp in request.path_to_goal.poses:
                pose_stamp.pose = do_transform_pose(pose_stamp.pose, transform)

                #TODO for students : Apply tranform on each Pose (of PoseStamped) with 'do_transform_pose' method out of tf.TransformListener()

                self.pathPoses.append(pose_stamp)

            self.get_logger().info("### New path with %d poses" % len(self.pathPoses))
            response.success.data = True

        except Exception as err:
            self.get_logger().info(str(err))
            response.success.data = False
            
        return response


    #******************************************************************************************
    #**********************************   GOALS COMPUTATION   *********************************
    #******************************************************************************************

    def shortest_angle_diff(self, th1, th2):
        """
            Returns the shortest angle between 2 angles in the trigonometric circle
        """        
        anglediff = fmod((th1 - th2), 2*pi)

        if anglediff < 0.0:
            if fabs(anglediff) > (2*pi + anglediff):
                anglediff = 2*pi + anglediff
        else:
            if anglediff > fabs(anglediff - 2*pi):
                anglediff = anglediff - 2*pi

        return anglediff

    def add_pose_2d_to_path(self, p, clear):
        """
            Add a pose p (type Pose2D) in the Path (self.pathPoses) to follow
            Argument clear (type bool) clears the existing Path (self.pathPoses) when set to true
        """   
        ps = PoseStamped()
        q = quaternion_from_euler(0.0, 0.0, p.theta)
        ps.header.frame_id = "odom"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = p.x
        ps.pose.position.y = p.y
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]   

        if clear:
            self.pathPoses.clear()
        self.pathPoses.append(ps)

    def compute_dist_angle(self):
        """
            Compute the Euclidean distance to the target and the angle between robot orientation and its target
            Return a tuple (distance, angle) with distance and angle to the target
        """           
        if len(self.pathPoses) > 0:
            
            
            #TODO for students : calculate distCurTarget and angle. To compute shortest angle use method shortestAngleDiff defined before


            return (distCurTarget, angle)
        else:
            return (0, 0)

    def compute_final_orientation(self):
        """
            Compute the final orientation based on the last waypoint orientation 
            Return an angle (float)
        """
        if len(self.pathPoses) > 0:


            #TODO for students : calculate angle . To compute shortest angle use method shortestAngleDiff defined before



            return angle       
        else:
            return 0

    def path_sequencer(self, dist, angle, finalOrientation):
        """
            Switch between the following sequences: "New Goal" <--> "Reach in progress" --> "Last Goal position Reached" --> "Last Goal pose (position + orientation) Reached"
            :return: One of the strings: "New Goal", "Reach in progress",  "Last Goal position Reached", "Last Goal pose (position + orientation) Reached"
        """

        if (dist < self.Waypoint_eps) and len(self.pathPoses) > 1:
            
            del self.pathPoses[0]
            # computeVelocity
            self.get_logger().info("# New goal : x=%f ; y=%f"  % (self.pathPoses[0].pose.position.x, self.pathPoses[0].pose.position.y))
            
            return ""   #TODO for students : return string matching with the state


        elif (dist < self.Destination_eps) and len(self.pathPoses) == 1:
            if fabs(finalOrientation) >= self.Angle_eps:
                state = ""  #TODO for students : return string matching with the state
            else:
                state = ""  #TODO for students : return string matching with the state

            self.get_logger().info("# %s : X = %.2f ; Y = %.2f "  % (state, self.pathPoses[0].pose.position.x, self.pathPoses[0].pose.position.y))
            return state

        return "Reach in progress"

    def compute_velocity(self, dist, angle, goalState):
        """
            Compute linear and angular velocity
            :param dist: Euclidean distance to the waypoint
            :param angle: Angle to the waypoint
            :param goalState: string giving the state from the path_sequencer method
            :rtype: geometry_msgs.msg/Twist
        """
        twist = Twist()

        twist.angular.z = 0 #TODO for students : Change 0 with gain and saturation (both ROSPARAM) applied to angle (as already done for linear velocity)

        if fabs(angle) < self.Angle_to_allow_linear:
            if goalState == "" or goalState == "":  #TODO for students : modify string matching with the state (help in pathSequencer docstring)  
                if self.isObstacle:
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = min(dist * self.K_linear, self.Sat_linear)   

        if goalState == "": #TODO for students : modify string matching with the state (help in pathSequencer docstring)
            twist.angular.z = 0.0

        return twist   

    def local_planning(self):
        """
            Main loop (20Hz):
                1/ Call method to compute distance and angle to waypoint
                2/ If last waypoint (goal), Call method to compute final orientation 
                3/ Call method to manage the sequence and get the goalState
                4/ Depending on the case:
                    a/ Call method to compute new distance and angle to waypoint
                    b/ Set angle to final orientation
                5/ Call method to compute velocity
                6/ Publish velocity
        """

        (dist, angle) = self.compute_dist_angle()

        if len(self.pathPoses) == 1:  
            finalOrientation = self.compute_final_orientation()
        else:
            finalOrientation = 0.0

        goalState = self.path_sequencer(dist, angle, finalOrientation)

        if goalState == "": #TODO for students : modify string matching with the state (help in pathSequencer docstring)        
            (dist, angle) = self.compute_dist_angle()

        elif goalState == "" or goalState == "":    #TODO for students : modify string matching with the state (help in pathSequencer docstring)        
            angle = finalOrientation

        twist = self.compute_velocity(dist, angle, goalState)
        self.velocity_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    local_planner = LocalPlanner()
    rclpy.spin(local_planner)
    local_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
