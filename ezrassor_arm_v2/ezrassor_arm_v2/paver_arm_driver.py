import std_msgs.msg
import rclpy

NODE = "paver_arm_driver"
GRIPPER_EXTERNAL_TOPIC = 'gripper_instructions'
GRIPPER_INTERNAL_TOPIC = "gripper_effort_controller/command"

QUEUE_SIZE = 10

publishers = {}

def handle_gripper_movements(data):
    publishers[GRIPPER_INTERNAL_TOPIC].publish(data.data)

def main(passed_args=None):

    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        publishers[GRIPPER_INTERNAL_TOPIC] = node.create_publisher(
            std_msgs.msg.Float64MultiArray, GRIPPER_INTERNAL_TOPIC, QUEUE_SIZE
        )

        node.create_subscription(
            std_msgs.msg.Float64MultiArray,
            GRIPPER_EXTERNAL_TOPIC,
            handle_gripper_movements,
            QUEUE_SIZE
        )

        node.get_logger().info("paver_arm_driver node created successfully")

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass