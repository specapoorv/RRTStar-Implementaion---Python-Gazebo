import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rrt_star import RRT
from rrt_star import Node as node
from rrt_star import main as run

class rrtPathFollower(Node):

    def __init__(self):
        super().__init__('rrt_path_follower')
        #self.get_logger().info('Running RRT* planner …')
        

        self.path=run()
        print(f"path is this {self.path}")
        #self.get_logger().info(self.path)

        self.x = 0 
        self.y = 0
        self.quaternion = None
        self.prev_error=0
        self.flag_1 = 1



        #self.get_logger().info(f'Planner returned {len(self.path)} points')
        self.pose=None
        self.odom_sub=self.create_subscription(Odometry,'/bcr_bot/odom',self.odomCallback,10)
        self.cmd_pub=self.create_publisher(Twist,'/bcr_bot/cmd_vel',10)
        self.create_timer(0.5, self.control_loop)
        self.new_node_pos=None
        self.x_node=0
        self.y_node=0
        self.vel=Twist()



    def odomCallback(self, msg):
        self.pose=msg.pose.pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.quaternion = msg.pose.pose.orientation

    def control_loop(self):
        if self.pose is None:
            return False
        if len(self.path)!=0 and self.new_node_pos is None:
            self.new_node_pos=self.path.pop(0)
            self.x_node=self.new_node_pos[0]
            self.y_node=self.new_node_pos[1]
            self.prev_error=0
            print(self.new_node_pos)
            #self.flag_1 = 1
            #while self.flag_1 == 1:

        elif self.new_node_pos is not None:
            print("Going towards node")
            self.move_to_node()

        else:
            print("Goal reached")
            self.vel.linear.x=0.0
            self.vel.angular.z=0.0
            self.vel_publish()

    def move_to_node(self):
        x=self.x
        y=self.y
        quaternion = self.quaternion
        yaw=math.atan2(2*(quaternion.w*quaternion.z + quaternion.x*quaternion.y), 1- 2*(quaternion.y**2 + quaternion.z**2))
        #vel=Twist()
        distance=math.sqrt((x-self.x_node)**2+(y-self.y_node)**2)
        angle=math.atan2(self.y_node-y,self.x_node-x)

        if abs(angle-yaw)>0.05:
            self.vel.angular.z= angle-yaw
            self.vel.linear.x = 0.0
        else:
            integral_error=distance*0.1
            derivative_error=(distance-self.prev_error)/0.1
            self.vel.angular.z = 0.0
            self.vel.linear.x=1.0*distance + 0.0*integral_error + 0.0*derivative_error
            #self.prev_error=distance

        if (distance<0.25):
            print("Node reached... Iterating to next node")
            self.new_node_pos = None

        
        self.vel_publish()

    def vel_publish(self):
        self.cmd_pub.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    node_1 = rrtPathFollower()
    rclpy.spin(node_1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




