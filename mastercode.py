import rrt_star
from get_occupancy_grid import Map_getter
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
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
        
        self.start_pos = (0, 0)
        self.goal_pos = (56, 2)
        
        self.step_size = 3.0
        self.search_radius = 7.0
        self.goal_radius = 5.0
        map = Map_getter()
        self.occupancy_grid = map.get_occupancy_grid()
        #print(self.occupancy_grid)
        self.reached = False
        self.waypoint_index = 0

    def get_path(self):
        self.map_size = (self.occupancy_grid.shape[1], self.occupancy_grid.shape[0])
        code = rrt_star(self.start_pos, self.goal_pos, self.map_size, self.step_size, self.search_radius, self.goal_radius, self.self.occupancy_grid)
        path = None

        if not code.is_valid_node(code.start_node):
            print("start node itself is invalid bro")
            return

        if not code.is_valid_node(code.goal_node):
            print('r u dumb, give me a goal thats not obstacle!')
            return
            
        
        code.develop_tree()
        max_iterations = 3000 ## CHANGE AS NEEDED


        print("iterating now")

        for i in range(max_iterations):
            print(f"iteration : {i}")
            code.develop_tree()

        self.path = code.path_generator()

        return path

    def get_next_waypoint(self, index):
        self.waypoint_wrt_rover = self.path[index] - self.path[index-1]
        return self.waypoint_wrt_rover



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
                self.X, self.Y = self.get_next_waypoint(self.waypoint_index)
                self.waypoint_index += 1
                self.reached = False
            return


        print(dist)
        self.velocity.publish(cmd)
                
    def abs_dist(self,msg):
        return np.sqrt((self.waypoint_wrt_rover[0]- msg.pose.pose.position.x)**2 + (self.waypoint_wrt_rover[1] - msg.pose.pose.position.y)**2)



def main(args = None):
    rclpy.init(args=args)
    node = Nav()
    rclpy.spin(node)
    rclpy.shutdown()
