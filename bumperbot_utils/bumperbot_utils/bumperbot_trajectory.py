#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped



class BumperbotTrajectory(Node):
    def __init__(self):
        super().__init__("Bumperbot_trajectory")
        self.path_msg_ = Path()
       
        self.path_pub_ = self.create_publisher(Path, "/bumperbot_controller/trajectory", 10)
        self.create_subscription(Odometry,"/bumperbot_controller/odom", self.odom_callback, 10)
        #debug
        self.get_logger().info("Trajectory node is created")


    def odom_callback(self, msg):
        self.path_msg_.header.frame_id = msg.header.frame_id
        # PoseStamped msg
        pose_msg_ = PoseStamped()
        pose_msg_.header.frame_id = msg.header.frame_id
        pose_msg_.header.stamp = msg.header.stamp
        pose_msg_.pose = msg.pose.pose
        
        #Trajectory

        self.path_msg_.poses.append(pose_msg_)
        self.path_pub_.publish(self.path_msg_)
        self.get_logger().info("Trajectory as nav_msgs/Path is being published")

    
    
def main():
    rclpy.init()
    node = BumperbotTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
