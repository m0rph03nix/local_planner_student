#!/usr/bin/env python3

__author__ = 'Raphael LEBER'

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from copy import deepcopy

from builtin_interfaces.msg import Time

from local_planner_srvs.srv import PathToGoal

class PathGenerator(Node):
    """
        Send a simple path to the service 
    """

    def __init__(self):
        super().__init__('testPathGenerator')
 
        self.cli = self.create_client(PathToGoal, 'pathService')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PathToGoal.Request()


    def send_request(self):
        self.req.path_to_goal = self.generatePath()

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()  


    def generatePath(self):
        pm = Path() 
        pm.header.frame_id = "map"
        pm.header.stamp = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.frame_id = pm.header.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()

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

def main(args=None):

    rclpy.init()

    pg = PathGenerator()

    response = pg.send_request()

    pg.get_logger().info(str(response.success))

    pg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        