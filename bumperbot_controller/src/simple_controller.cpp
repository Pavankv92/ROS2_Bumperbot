#include "bumperbot_controller/simple_controller.h"
#include <Eigen/Geometry>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name) : Node(name)
{

    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Wheel radius: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Wheel separation: " << wheel_separation_);

    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10,
                                                                     std::bind(&SimpleController::velocity_callback, this, _1));

    speed_conversion_ << wheel_radius_ / 2.0, wheel_radius_ / 2.0, wheel_radius_ / wheel_separation_, -wheel_radius_ / wheel_separation_;

    RCLCPP_INFO_STREAM(get_logger(), "Conversion matrix:" << speed_conversion_);
}

void SimpleController::velocity_callback(const geometry_msgs::msg::TwistStamped &msg)
{

    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg ;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1)); //Right wheel
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0)); //Left wheel
    wheel_cmd_pub_->publish(wheel_speed_msg);

}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}