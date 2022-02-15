# Node for the paver arm drivers
# Includes the following controller's drivers:
#   gripper_effort_controller

import std_msgs.msg
import rclpy

# Initalize neccessary variables
NODE = "paver_arm_driver"
GRIPPER_EXTERNAL_TOPIC = 'gripper_instructions'
GRIPPER_INTERNAL_TOPIC = "gripper_effort_controller/command"
QUEUE_SIZE = 10
publishers = {}

# Callback function for handling the gripper movements and publishing to the controller
def handle_gripper_movements(data):
    publishers[GRIPPER_INTERNAL_TOPIC].publish(data.data)

# Main function
def main(passed_args=None):

    try:
        # Initalize rclpy and create the node
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Initalize all needed publishers
        publishers[GRIPPER_INTERNAL_TOPIC] = node.create_publisher(
            std_msgs.msg.Float64MultiArray, GRIPPER_INTERNAL_TOPIC, QUEUE_SIZE
        )

        # Create a subscription for each controller that this node controls
        node.create_subscription(
            std_msgs.msg.Float64MultiArray,
            GRIPPER_EXTERNAL_TOPIC,
            handle_gripper_movements,
            QUEUE_SIZE
        )

        # Log that the node was successful
        node.get_logger().info("paver_arm_driver node created successfully")

        # Spin to await data from the subscribed topics
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass