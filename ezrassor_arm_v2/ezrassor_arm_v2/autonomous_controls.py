import sys
import rclpy
from rclpy.node import Node
from std_msgs.msgs import Float64
from geometry_msgs.msg import Twist



# Topics
topics = [
    'front_drum_instructions',
    'back_drum_instructions',
    'front_arm_instructions',
    'back_arm_instructions',
    'wheels_instructions',
    'gripper_instructions'
]

class StandardRoverController(Node):
    def __init__(self, topics, args):

        super().__init__("standard_rover_controller")

        #Controllers Topics
        self.front_drum_instructions_topic = topics[0],
        self.back_drum_instructions_topic = topics[1],
        self.front_arm_instructions_topic = topics[2],
        self.back_arm_instructions_topic = topics[3],
        self.wheels_instructions_topic = topics[4],

        #Controller Settings
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 0.5
        self.obstacle_threshold = 4.0
        self.obstacle_buffer = 0.1
        self.move_increment = 1.5
        self.max_obstacle_angle = 45.0
        self.min_hole_diameter = 3.0
        self.queue_size = 10

        #Controller Publishers For Driver Nodes
        self.front_drum_pub = self.create_publisher(
            Float64, self.front_drum_instructions_topic, self.queue_size
        )

        self.back_drum_pub = self.create_publisher(
            Float64, self.back_drum_instructions_topic, self.queue_size
        )

        self.front_arm_pub = self.create_publisher(
            Float64, self.front_arm_instructions_topic, self.queue_size
        )

        self.back_arm_pub = self.create_publisher(
            Float64, self.back_arm_instructions_topic, self.queue_size
        )

        self.wheels_pub = self.create_publisher(
            Twist, self.wheels_instructions_topic, self.queue_size
        )

        #Controller Subscribers
        self.auto_command_sub = self.create_subscription(
            Float64, 'auto_command', self.auto_command_callback, 1
        )
    
    def auto_command_callback(data):
        print()


class PaverArmRoverController(Node):
    def __init__(self, topics):
        self.gripper_instructions_topic = topics[5]     


def main(args=None):
    rclpy.init(args=args)
    standard_controller_args = sys.argv[1:10]

    if (standard_controller_args[0] == 'arm'):
        controller_arm = PaverArmRoverController(topics=topics)
        controller = StandardRoverController(topics=topics, args=standard_controller_args)
    else:
        controller = StandardRoverController(topics=topics, args=standard_controller_args)