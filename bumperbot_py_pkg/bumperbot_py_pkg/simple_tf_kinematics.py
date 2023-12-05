import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform


class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)

        # dynamic
        self.dynamic_tf_msg_ = TransformStamped()
        self.x_increment_ = 0.05
        self.last_x_ = 0.0
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # static
        self.static_tf_msg_ = TransformStamped()
        self.static_tf_msg_.header.stamp = self.get_clock().now().to_msg()
        self.static_tf_msg_.header.frame_id = "bumperbot_base"
        self.static_tf_msg_.child_frame_id = "bumperbot_top"
        self.static_tf_msg_.transform.translation.x = 0.0
        self.static_tf_msg_.transform.translation.y = 0.0
        self.static_tf_msg_.transform.translation.z = 0.3
        self.static_tf_msg_.transform.rotation.x = 0.0
        self.static_tf_msg_.transform.rotation.y = 0.0
        self.static_tf_msg_.transform.rotation.z = 0.0
        self.static_tf_msg_.transform.rotation.w = 1.0

        self.static_tf_broadcaster_.sendTransform(self.static_tf_msg_)

        self.get_logger().info(
            f"Publishing static fransform between {self.static_tf_msg_.header.frame_id} and {self.static_tf_msg_.child_frame_id}")

        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.get_tf_srv_ = self.create_service(
            GetTransform, "get_transform", self.get_tf_callback)

    def timer_callback(self):
        self.dynamic_tf_msg_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf_msg_.header.frame_id = "odom"
        self.dynamic_tf_msg_.child_frame_id = "bumperbot_base"
        self.dynamic_tf_msg_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_tf_msg_.transform.translation.y = 0.0
        self.dynamic_tf_msg_.transform.translation.z = 0.0
        self.dynamic_tf_msg_.transform.rotation.x = 0.0
        self.dynamic_tf_msg_.transform.rotation.y = 0.0
        self.dynamic_tf_msg_.transform.rotation.z = 0.0
        self.dynamic_tf_msg_.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_tf_msg_)

        self.last_x_ = self.dynamic_tf_msg_.transform.translation.x

    def get_tf_callback(self, request, response):
        self.get_logger().info(
            f"Requested transform between {request.frame_id}, and {request.child_frame_id}")
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(
                request.frame_id, request.child_frame_id, rclpy.time.Time())

        except TransformException as e:
            self.get_logger.error(
                f"Error occured while accessing fransform between {self.static_tf_msg_.header.frame_id} and {self.static_tf_msg_.child_frame_id}")
            response.success = False

        response.transform = requested_transform
        response.success = True
        return response


def main():
    rclpy.init()
    node = SimpleTfKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
