#!/usr/bin/env python3

from rrt_star_final import RRT
from rrt_star_final import visualize_tree
from get_occupancy_grid import Map_getter
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt

class Nav(Node):
    def __init__(self, x, y):

        super().__init__("feed")
        self.position = self.create_subscription(Odometry, "/bcr_bot/odom", self.gothere_callback, 10)
        self.velocity = self.create_publisher(Twist, '/bcr_bot/cmd_vel' , 10)
        
        self.start_pos = (45, 68)
        self.goal_pos = (x, y)
        
        self.step_size = 10.0
        self.search_radius = 20.0
        self.goal_radius = 5.0
        map = Map_getter()
        self.occupancy_grid = map.get_occupancy_grid()
        
        self.reached = False
        self.waypoint_index = 0
        self.just_started = True
        self.max_iterations = 3000   ## CHANGE AS NEEDED
        self.waypoint_wrt_rover = None
        self.get_path()

    def get_path(self):
        self.map_size = (self.occupancy_grid.shape[1], self.occupancy_grid.shape[0])
        obj = RRT(
            self.start_pos, 
            self.goal_pos, 
            self.map_size, 
            self.step_size, 
            self.search_radius, 
            self.goal_radius, 
            self.occupancy_grid
            )
        
        self.initial_path = None

        if not obj.is_valid_node(obj.start_node):
            print("start node itself is invalid bro")
            return

        if not obj.is_valid_node(obj.goal_node):
            print('r u dumb, give me a goal thats not obstacle!')
            return
            
        obj.develop_tree()
        print("iterating now")

        for i in range(self.max_iterations):
            print(f"iteration : {i}")
            obj.develop_tree()

        self.initial_path = obj.path_generator()
        print(self.initial_path)

        self.path = []
        for i in range(len(self.initial_path)):
            x, y = self.initial_path[i]
            self.path.append((x - 45, y - 68))
        visualize_tree(obj.node_tree, self.occupancy_grid, obj.goal_node, obj.start_node, self.initial_path)
        plt.show() ## for vis the path
        print(self.path)

        # return self.path
    

    def get_next_waypoint(self, index):
        x1, y1 = self.path[index]
        x0, y0 = self.path[index - 1]
        self.waypoint_wrt_rover = (x1 - x0, y1 - y0)
        return self.waypoint_wrt_rover



    def gothere_callback(self, msg):
        # if self.just_started == True:
        #     self.start_pos = msg.pose.pose.position
        #     self.just_started = False
        cmd = Twist()

        if self.waypoint_wrt_rover is None:
            self.X, self.Y = self.get_next_waypoint(self.waypoint_index)
            self.waypoint_index += 1
            return
    
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
        
        # cmd.linear.x = 0.4 * dist

        angle_to_goal = np.arctan2(self.Y - self.coordinate.y, self.X - self.coordinate.x)
        angle_diff = angle_to_goal - yaw
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff)) 
        cmd.angular.z = angle_diff


        if abs(angle_diff-yaw)>0.1:
            cmd.angular.z= 1.0
            cmd.linear.x = 0.0

        else:
            # integral_error=dist*0.1
            # derivative_error=(dist-self.prev_error)/0.1
            cmd.angular.z = 0.0
            cmd.linear.x=1.0 #+ 0.0*integral_error + 0.0*derivative_error
            #self.prev_error=distance

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
        
        if self.waypoint_wrt_rover is None:
            return
    
        return np.sqrt((self.waypoint_wrt_rover[0]- msg.pose.pose.position.x)**2 + (self.waypoint_wrt_rover[1] - msg.pose.pose.position.y)**2)



def main(args = None):
    rclpy.init(args=args)
    x = int(input("Enter the x coordinate of the goal:"))
    y = int(input("Enter the y coordinate of the goal:"))
    node = Nav(x, y)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
