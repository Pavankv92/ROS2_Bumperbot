#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Imu outputs angular velocity, using kalman filter to estimate the current velocity by fusing wheel encoder data and imu
# Odometry contains both position and velocity

class KalmanFiler(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odom_callback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imu_callback, 10)
        self.kalman_odom_pub_ = self.create_publisher(Odometry, "bumnperbot_controller/odom_kalman", 10)

        # states, angular velocity
        self.mean_ = 0.0
        self.varaince_ = 1000.0
       
        self.imu_angular_z = 0.0
        self.is_first_odom_ = True
        self.last_angular_z = 0.0

        self.motion_ = 0.0
        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5
        self.kalman_odom_ = Odometry()
    
    def imu_callback(self, msg):
        self.imu_angular_z = msg.angular_velocity.z
    
    def measurement_update(self):

        self.mean_ =  (self.measurement_variance_ * self.mean_ + self.varaince_ * self.imu_angular_z ) / (self.measurement_variance_ + self.varaince_)
        self.varaince_ = (self.measurement_variance_ * self.varaince_) / (self.measurement_variance_ + self.varaince_)
    
    def state_prediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.varaince_ = self.varaince_ + self.motion_variance_

    def odom_callback(self, msg):
        self.kalman_odom_ = msg

        if self.is_first_odom_:
            self.mean_ = msg.twist.twist.angular.z 
            self.last_angular_z = msg.twist.twist.angular.z 
            self.is_first_odom_ = False
            return
        
        self.motion_ = msg.twist.twist.angular.z - self.last_angular_z

        self.state_prediction()
        self.measurement_update()
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.kalman_odom_pub_.publish(self.kalman_odom_)
        
def main():
    rclpy.init()
    node = KalmanFiler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()