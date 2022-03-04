import queue
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msgs import Float64
from geometry_msgs.msg import (Twist, Point)
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import (JointState, Imu, LaserScan)
import actionlib

import auto_functions as af
import utility_functions as uf

# Topics
topics = [
    'wheels_instructions',
    'front_arm_instructions',
    'back_arm_instructions',
    'front_drum_instructions',
    'back_drum_instructions',
    'move_group_interface/command'
]

class StandardRoverController(Node):
    def __init__(self, topics, args):

        super().__init__("standard_rover_controller")

        self.rover_model = args[0]
        self.target = Point()
        self.target.x = args[1]
        self.target.y = args[2]
        self.spawn = Point()
        self.spawn.x = args[3]
        self.spawn.y = args[4]
        self.odom_flag = args[5]
        self.park_flag = args[6]
        self.swarm_flag = args[7]
        self.world = args[8]

        #Controller Settings
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 0.5
        self.obstacle_threshold = 4.0
        self.obstacle_buffer = 0.1
        self.move_increment = 1.5
        self.max_obstacle_angle = 45.0
        self.min_hole_diameter = 3.0
        self.queue_size = 10

        self.namespace = self.get_namespace()

        self.world_state = uf.WorldState(self)

        self.ros_util = uf.ROSUtility(
            self,
            self.rover_model,
            topics[0],
            topics[1],
            topics[2],
            topics[3],
            topics[4],
            topics[5],
            self.max_linear_velocity,
            self.max_angular_velocity,
            self.obstacle_threshold,
            self.obstacle_buffer,
            self.move_increment
        )

        if self.odom_flag:

            self.world_state.inital_spawn(self.spawn.x, self.spawn.y)

            self.odom_sub = self.create_subscription(
                Odometry, 'odometry/filtered', self.world_state.odometryCallBack
            )
        
        else:
            self.sim_state_sub = self.create_subscription(
                LinkStates, '/gazebo/link_states', self.world_state.simStateCallBack
            )
        
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.world_state.imuCallBack
        )

        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.world_state.jointCallBack
        )

        self.auto_command_sub = self.create_subscription(
            Float64, 'auto_command', self.ros_util.auto_command_callback, 1
        )

        self.obstacle_sub = self.create_subscription(
            LaserScan, 'obstacle_detection/combined', uf.on_scan_update, queue_size = 1
        )

        if self.swarm_flag:

            self.ros_util.auto_function_command = 16

            # TBC
        
        else:
            self.world_state.target_location = self.target
            self.world_state.dig_site = self.target
        
    #def send_status(self, request):
    
    #def execute_action(self, goal):

    #def preempt_cb(self):

    def full_autonomy(self, world_state, ros_util):

        world_state.logger.info(
            'Full autonomy activated'
        )

        if (self.rover_model != 'arm'):
            while ros_util.auto_function_command == 16:
                af.auto_drive_location(world_state, ros_util)
                if (ros_util.auto_function_command != 16):
                    break
                af.auto_dig(world_state, ros_util, 7)
                if (ros_util.auto_function_command != 16):
                    break
                af.auto_dock(world_state, ros_util, 4)
                if (ros_util.auto_function_command != 16):
                    break
                af.auto_dump(world_state, ros_util, 4)
                world_state.target_location = world_state.dig_site
        else:
            while ros_util.auto_function_command == 16:
                af.auto_drive_location(world_state, ros_util)
                if (ros_util.auto_function_command != 16):
                    break
                
                # TBC
    
    def autonomous_control_loop(self, world_state, ros_util):

        while True:
            while(
                ros_util.auto_function_command == 0
                or ros_util.auto_function_command == 32
            ):
                ros_util.publish_actions(uf.actions)
                ros_util.rate.sleep()

            ros_util.control_pub.publish(True)

            if ros_util.auto_function_command == 1:
                af.auto_drive_location(world_state, ros_util)
            elif ros_util.auto_function_command == 2:
                af.auto_dig(world_state, ros_util, 10)
            elif ros_util.auto_function_command == 4:
                af.auto_dump(world_state, ros_util, 4)
            elif ros_util.auto_function_command == 8:
                af.auto_dock(world_state, ros_util)
            elif ros_util.auto_function_command == 16:
                self.full_autonomy(world_state, ros_util)
            else:
                world_state.logger.info(
                    'Invalid auto-function request: {}'.format(ros_util.auto_function_command)
                )
        
            ros_util.auto_function_command = 0
            ros_util.publish_actions(uf.actions)
            ros_util.control_pub.publish(False)
     


def main(args=None):
    rclpy.init(args=args)
    standard_controller_args = sys.argv[1:10]
    controller = StandardRoverController(topics=topics, args=standard_controller_args)

    if not sys.argv[8]:
        controller.autonomous_control_loop(
            controller.world_state, controller.ros_util
        )
    