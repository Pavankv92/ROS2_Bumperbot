#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

void imu_callback(const sensor_msgs::msg::Imu &imu_msg)
{
    sensor_msgs::msg::Imu new_imu_msg;
    new_imu_msg = imu_msg ;
    new_imu_msg.header.frame_id = "base_footprint" ;
    imu_pub->publish(new_imu_msg);
}

int main(int argc, char* argv[])
{
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_republisher");
    rclcpp::sleep_for(1s);
    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_ekf", 10);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("imu/out", 10, imu_callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}