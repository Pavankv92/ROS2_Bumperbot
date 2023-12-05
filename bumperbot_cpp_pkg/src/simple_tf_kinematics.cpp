#include "bumperbot_cpp_pkg/simple_tf_kinematics.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : Node(name)
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    static_tf_msg_.header.frame_id = "bumperbot_base";
    static_tf_msg_.child_frame_id = "bumperbot_top";
    static_tf_msg_.header.stamp = get_clock()->now();
    static_tf_msg_.transform.translation.x = 0.0;
    static_tf_msg_.transform.translation.y = 0.0;
    static_tf_msg_.transform.translation.z = 0.3;
    static_tf_msg_.transform.rotation.x = 0.0;
    static_tf_msg_.transform.rotation.y = 0.0;
    static_tf_msg_.transform.rotation.z = 0.0;
    static_tf_msg_.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_tf_msg_);

    RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform between" << static_tf_msg_.header.frame_id << "and" << static_tf_msg_.child_frame_id);
    timer_ = create_wall_timer(0.1s, std::bind(&SimpleTfKinematics::timer_callback, this));

    get_transform_srv_ = create_service<bumperbot_msgs::srv::GetTransform>("get_transform", std::bind(&SimpleTfKinematics::get_tf_callback, this, _1, _2));
}

void SimpleTfKinematics::timer_callback()
{
    dynamic_tf_msg_.header.stamp = get_clock()->now();
    dynamic_tf_msg_.header.frame_id = "odom";
    dynamic_tf_msg_.child_frame_id = "bumperbot_base";
    dynamic_tf_msg_.transform.translation.x = last_x_ + x_increment_;
    dynamic_tf_msg_.transform.translation.y = 0.0;
    dynamic_tf_msg_.transform.translation.z = 0.0;
    dynamic_tf_msg_.transform.rotation.x = 0.0;
    dynamic_tf_msg_.transform.rotation.y = 0.0;
    dynamic_tf_msg_.transform.rotation.z = 0.0;
    dynamic_tf_msg_.transform.rotation.w = 1.0;
    dynamic_tf_broadcaster_->sendTransform(dynamic_tf_msg_);
    last_x_ = dynamic_tf_msg_.transform.translation.x;
}

bool SimpleTfKinematics::get_tf_callback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> request,
                                         const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> response)
{
    RCLCPP_INFO_STREAM(get_logger(), "Requested transform between " << request->frame_id << " and " << request->child_frame_id);
    geometry_msgs::msg::TransformStamped requested_transform;
    try
    {
        requested_transform = tf_buffer_->lookupTransform(request->frame_id, request->child_frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in accessing transform between " << request->frame_id << "and" << request->child_frame_id << ": " << ex.what());
        response->success = false;
        return true;
    }
    response->transform = requested_transform;
    response->success = true;
    return true;
}

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
