#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

class KalmanFilter : public rclcpp::Node
{
    public:
        KalmanFilter(const std::string &name);
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ ;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ ;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kalman_odom_pub_;

        double mean_;
        double variance_ ;
        double imu_angular_z_ ;
        bool is_first_odom_ ;
        double last_angular_z_ ;
        double motion_ ;
        nav_msgs::msg::Odometry kalman_odom_;

        double motion_variance_;
        double measurment_variance_;

        void odom_callback(const nav_msgs::msg::Odometry &odom_msg);
        void imu_callback(const sensor_msgs::msg::Imu &imu_msg);
        void measurement_update();
        void state_prediction();

} ;