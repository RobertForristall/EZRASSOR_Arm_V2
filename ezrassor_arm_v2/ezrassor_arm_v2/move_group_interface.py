from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
from ezrassor_arm_interfaces.msg import ArmCommand

from pymoveit2 import MoveIt2
from ezrassor_arm_v2 import arm

class MoveGroupInterface(Node):

    def __init__(self):

        super().__init__("move_group_interface")
        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=arm.joint_names(),
            base_link_name=arm.base_link_name(),
            end_effector_name=arm.end_effector_name(),
            group_name=arm.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        self.sub_topic = 'move_group_interface/command'
        self.gripper_pub_topic = 'gripper_instructions'

        self.gripper_pub = self.create_publisher(
            Float64MultiArray, self.gripper_pub_topic, 10
        )

        self.test_command_pub = self.create_publisher(
            ArmCommand, self.sub_topic, 10
        )

        self.subscriber = self.create_subscription(
            ArmCommand,
            self.sub_topic,
            self.handle_command,
            10
        )

        self.test_sub = self.create_subscription(
            String,
            'arm_test',
            self.test_arm_msg,
            10
        )

        self.temp = ArmCommand()
        self.temp.command = "home"
        self.temp.pose_data = [1.0, 0.3, 0.3, 0.0, 0.0, 0.0, 1.0]
        self.temp.joint_data = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    def handle_command(self, data):
        if (data.command == "home"):
            self.move_to_home()

        elif(data.command == 'place'):
            self.move_to_pose(data.pose_data)

        elif(data.command == 'full_place_1st'):
            self.move_to_prep()
            self.move_to_first_pickup()
            self.move_to_post_pickup()
            self.move_to_home()
            self.move_to_pose(data.pose_data)
            self.move_to_home()
        else:
            self.get_logger().info(
                "Failed to read command"
            )

    def test_arm_msg(self, data):

        self.get_logger().info(
            "Publishing to move_group_interface/command"
        )

        self.test_command_pub.publish(self.temp)

    # data.data = [command, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
    def move_to_pose(self, data):
        position = data[0:3]
        quat = data[3:]

        self.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat)}}}"
        )
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat)
        self.moveit2.wait_until_executed()
    
    def move_to_home(self):
        home_pose = arm.home_pose()
        self.get_logger().info(
            "Moving to home position"
        )
        self.moveit2.move_to_configuration(joint_positions=home_pose)
        self.moveit2.wait_until_executed()

    def move_to_prep(self, data):
        prep_pose = arm.prep_pose()
        self.get_logger().info(
            "Moving to prep position"
        )
        self.moveit2.move_to_configuration(joint_positions=prep_pose)
        self.moveit2.wait_until_executed()

    def open_gripper(self):
        self.gripper_pub.publish(Float64MultiArray([-1.0, -1.0]))

    def close_gripper(self):
        self.gripper_pub.publish(Float64MultiArray([0.0, 0.0]))
        

    def move_to_first_pickup(self, data):
        first_pickup_pose = arm.first_pickup()
        self.get_loggr().info(
            "Picking up first paver"
        )
        self.moveit2.move_to_configuration(joint_positions=first_pickup_pose)
        self.moveit2.wait_until_executed()

        self.close_gripper()
        self.get_logger().info(
            "Grippers closed"
        )

    def move_to_post_pickup(self, data):
        post_pickup_pose = arm.post_pickup()
        self.get_logger().info(
            "Moving to post pickup pose"
        )
        self.moveit2.move_to_configuration(joint_positions=post_pickup_pose)
        self.moveit2.wait_until_executed()


def main(args=None):

    try:
        rclpy.init(args=args)

        # Create node for this example
        move_group_interface = MoveGroupInterface()

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(move_group_interface)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        rclpy.spin(move_group_interface)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()