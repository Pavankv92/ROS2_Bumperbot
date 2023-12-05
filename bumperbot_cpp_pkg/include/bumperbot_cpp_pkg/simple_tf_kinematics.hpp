# pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <bumperbot_msgs/srv/get_transform.hpp>

#include <memory>


class SimpleTfKinematics : public rclcpp::Node{
    public:
        SimpleTfKinematics(const std::string &name);

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> dynamic_tf_broadcaster_;
        geometry_msgs::msg::TransformStamped static_tf_msg_ ;
        geometry_msgs::msg::TransformStamped dynamic_tf_msg_ ;

        rclcpp::Service<bumperbot_msgs::srv::GetTransform>::SharedPtr get_transform_srv_ ;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        rclcpp::TimerBase::SharedPtr timer_ ;
        double x_increment_{0.05};
        double last_x_ ;

        void timer_callback();
        bool get_tf_callback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> request,
        const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> response);



};