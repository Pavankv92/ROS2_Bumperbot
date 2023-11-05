#include <rclcpp/rclcpp.hpp>
#include "bumperbot_cpp_pkg/turtlesim_kinematics.h"

using std::placeholders::_1;

TurtlesimKinematics::TurtlesimKinematics(const std::string &name) : Node(name)
{
    turtle1_pose_sub_ = create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtlesimKinematics::turtle1PoseCallback, this, _1));
    turtle2_pose_sub_ = create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&TurtlesimKinematics::turtle2PoseCallback, this, _1));
}

void TurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose &pose)
{
    turtle1_last_pose_ = pose;
}
void TurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose)
{
    turtle2_last_pose_ = pose;
    float Tx = turtle2_last_pose_.x - turtle1_last_pose_.x;
    float Ty = turtle2_last_pose_.y - turtle1_last_pose_.y;
    float theta_rad = turtle2_last_pose_.theta - turtle1_last_pose_.theta;
    float theta_deg = theta_rad * 180 / 3.14;

    RCLCPP_INFO_STREAM(get_logger(), "\nTranslation vector turtle2 -> turtle1 \n"<< 
        "Tx: " << Tx << "\n"<< 
        "Ty: " << Ty << "\n"<<
        "Rotation matrix turtle2 -> turtle1 \n"<< 
        "Theta(rad)" << theta_rad << "\n"<< 
        "Theta(deg)" << theta_deg << "\n"<< 
        "|R11       R12|:  |" << std::cos(theta_rad) << "\t" << -std::sin(theta_rad) << "|\n" << 
        "|R21       R22|:  |" << std::sin(theta_rad) << "\t" <<  std::cos(theta_rad) << "|\n"
    );
}

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimKinematics>("Simple_turtlesim_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}