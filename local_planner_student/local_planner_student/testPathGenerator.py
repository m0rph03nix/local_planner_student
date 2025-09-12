#!/usr/bin/env python3

__author__ = 'Raphael LEBER'

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from copy import deepcopy

from local_planner_srvs.srv import PathToGoal


class PathGenerator(Node):
    """
        Send a simple path to the service 
    """

    def __init__(self):
        super().__init__('testPathGenerator')
 
        self.cli = self.create_client(PathToGoal, 'pathService')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PathToGoal.Request()

    def send_request(self):
        self.req.path_to_goal = self.generatePath()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()  

    def generatePath(self):
        pm = Path() 
        pm.header.frame_id = "odom"
        pm.header.stamp = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.frame_id = pm.header.frame_id

        # Définition des points du chemin (x, y)
        # points = [
        #     (2.8, 2.8),
        #     (4.0, 3.0),
        #     (8.0, 6.0),
        #     (4.0, 3.0),
        #     (2.8, 2.8),
        # ]

        # points = [
        #     (3.5, -0.5),
        #     (3.5, -2.0),
        #     (-3, -2.0),
        #     (-3, -3.5),
        #     (-3, -2.0),
        #     (3.5, -2.0),
        #     (3.5, -0.5),

        # ]        


        points = [
            (-3.2, -0.5),
            (-3.2, -2.8),
            (-0.5, -2.8),
            (-0.5, -4.2),
            (-0.5, -2.8),
            (-3.2, -2.8),
            (-3.2, -0.5),


        ]           

        for x, y in points:
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            pm.poses.append(deepcopy(ps))

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
