from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

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
        self.sub_topic = 'move_group_interface/command'

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            self.sub_topic,
            self.move_to_pose,
            10
        )
    # data.data = [command, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
    def move_to_pose(self, data):
        position = data.data[1:4]
        quat = data.data[4:]

        self.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat)}}}"
        )
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat)
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