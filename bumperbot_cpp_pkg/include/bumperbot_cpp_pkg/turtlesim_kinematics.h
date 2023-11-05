#ifndef TURTLESIM_KINEMATICS_H
#define TURTLESIM_KINEMATICS_H

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class TurtlesimKinematics : public rclcpp::Node{

    public:
        TurtlesimKinematics(const std::string &name);
    private:
        void turtle1PoseCallback(const turtlesim::msg::Pose &pose);
        void turtle2PoseCallback(const turtlesim::msg::Pose &pose);
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

        turtlesim::msg::Pose turtle1_last_pose_;
        turtlesim::msg::Pose turtle2_last_pose_;

};

#endif