import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

import numpy as np

class Nav(Node):
    def __init__(self):



        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)


        super().__init__("feed")
        self.position = self.create_subscription(
            Odometry, "/bcr_bot/odom", self.gothere_callback, 10)
        self.velocity = self.create_publisher(Twist, '/bcr_bot/cmd_vel' , 10)
        
        self.destination()
        self.reached = False


    def gothere_callback(self, msg):

        cmd = Twist()
      
        dist = self.abs_dist(msg)
        self.coordinate = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        qx = quat.x
        qy = quat.y
        qz = quat.z
        qw = quat.w

        # Compute yaw (in radians)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz))
        
        cmd.linear.x = 0.4 * dist


        angle_to_goal = np.arctan2(self.Y - self.coordinate.y, self.X - self.coordinate.x)
        angle_diff = angle_to_goal - yaw
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff)) 
        cmd.angular.z = angle_diff

        if dist < 0.025:
            twist = Twist()
            self.velocity.publish(twist) 
            if not self.reached:
                print(dist)
                self.reached = True
            return


        print(dist)
        self.velocity.publish(cmd)



    def destination(self):
        self.X = float(input("Enter the final destination's X coord:"))
        self.Y = float(input("Enter the final destination's Y coord:"))

    def abs_dist(self,msg):
        
        return np.sqrt((self.X- msg.pose.pose.position.x)**2 + (self.Y - msg.pose.pose.position.y)**2)
def main(args = None):
    rclpy.init(args=args)
    node = Nav()
    rclpy.spin(node)
    rclpy.shutdown()
