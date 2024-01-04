#include "bumperbot_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string &name)
    : Node(name), 
      mean_(0.0), 
      variance_(1000.0),
      imu_angular_z_(0.0),
      is_first_odom_(true),
      last_angular_z_(0.0),
      motion_(0.0),
      motion_variance_(4.0),
      measurment_variance_(0.5)
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odom_callback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/out", 10, std::bind(&KalmanFilter::imu_callback, this, _1));
    kalman_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("bumperbot_controller/odom_kalman", 10);

}

 void KalmanFilter::odom_callback(const nav_msgs::msg::Odometry &odom_msg)
{
    kalman_odom_ = odom_msg;

    if (is_first_odom_) 
    {   
        mean_ = odom_msg.twist.twist.angular.z;
        last_angular_z_ = odom_msg.twist.twist.angular.z;
        is_first_odom_ = false;
        return;
    }
    motion_ = odom_msg.twist.twist.angular.z - last_angular_z_;
    
    state_prediction();
    measurement_update();

    kalman_odom_.twist.twist.angular.z = mean_;
    kalman_odom_pub_->publish(kalman_odom_);
}

void KalmanFilter::state_prediction()
{
    mean_ = mean_ + motion_ ;
    variance_ = variance_ + motion_variance_ ;
}

void KalmanFilter::measurement_update(){
    mean_ = (measurment_variance_ * mean_ + variance_ * imu_angular_z_) / (measurment_variance_ + variance_);
    variance_ = (measurment_variance_ * variance_) / (measurment_variance_ + variance_);
}


void KalmanFilter::imu_callback(const sensor_msgs::msg::Imu &imu_msg)
{
    imu_angular_z_ = imu_msg.angular_velocity.z ;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}