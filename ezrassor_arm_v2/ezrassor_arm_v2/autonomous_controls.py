import queue
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import (Float64, Bool)
from geometry_msgs.msg import (Twist, Point)
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import (JointState, Imu, LaserScan)
from threading import Thread

import ezrassor_arm_v2.auto_functions as af
import ezrassor_arm_v2.utility_functions as uf

# Topics
topics = [
    'wheel_instructions',
    'front_arm_instructions',
    'back_arm_instructions',
    'front_drum_instructions',
    'back_drum_instructions',
    'move_group_interface/command'
]

base_namespace = 'ezrassor/'

class StandardRoverController(Node):
    def __init__(self, topics, args):

        super().__init__("standard_rover_controller")

        self.rover_model = args[0]
        self.target = Point()
        self.target.x = float(args[1])
        self.target.y = float(args[2])
        self.spawn = Point()
        self.spawn.x = float(args[3])
        self.spawn.y = float(args[4])
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

        self.get_logger().info(
                'Swarm Flag: {}'.format(self.swarm_flag)
            )

        self.world_state.target_location.x = self.target.x
        self.world_state.target_location.y = self.target.y
        self.world_state.dig_site = self.target

        self.get_logger().info(
            'Target Location: {}'.format(self.world_state.target_location)
        )

        #if self.swarm_flag:
        #
        #    self.ros_util.auto_function_command = 16
        #
        #    # TBC
        #
        #else:
        #    self.world_state.target_location.x = self.target.x
        #    self.world_state.target_location.y = self.target.y
        #    self.world_state.dig_site = self.target
        #
        #    self.get_logger().info(
        #        'Target Location: {}'.format(self.world_state.target_location)
        #    )

        self.joint_state_sub = self.create_subscription(
            JointState, 'ezrassor/joint_states', self.jointCallBack, 1
        )

        if self.odom_flag:

            self.world_state.inital_spawn(self.spawn.x, self.spawn.y)

            self.odom_sub = self.create_subscription(
                Odometry, 'odometry/filtered', self.world_state.odometryCallBack, 1
            )
        
        else:
            # TBC Add link_states plugin
            self.sim_state_sub = self.create_subscription(
                LinkStates, '/gazebo/link_states', self.world_state.simStateCallBack, 1
            )
        
        self.imu_sub = self.create_subscription(
            Imu, base_namespace + 'imu_data', self.world_state.imuCallBack, 1
        )

        self.auto_command_sub = self.create_subscription(
            Float64, 'auto_command', self.autonomous_control_loop, 1
        )

        self.obstacle_sub = self.create_subscription(
            LaserScan, 'obstacle_detection/combined', uf.on_scan_update, 1
        )

            
        
    #def send_status(self, request):
    
    #def execute_action(self, goal):

    #def preempt_cb(self):

    def jointCallBack(self, data):

        front_index = data.name.index('arm_front_joint')
        back_index = data.name.index('arm_back_joint')

        self.world_state.arm_angles['front'] = data.position[front_index]
        self.world_state.arm_angles['back'] = data.position[back_index]

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
    
    # This function is used to test speciffic drivers and callbacks
    def test_function(self, world_state, ros_util):

        msg = Twist()
        msg.angular.z = 1.0
        ros_util.movement_pub.publish(msg)
    
    def autonomous_control_loop(self, msg):

        self.get_logger().info(
            'Reading auto function command'
        )

        self.ros_util.auto_function_command = msg.data

        while True:
            while(
                self.ros_util.auto_function_command == 0
                or self.ros_util.auto_function_command == 32
            ):
                self.ros_util.publish_actions(uf.actions)
                self.ros_util.rate.sleep()

            temp = Bool()
            temp.data = True
            self.ros_util.control_pub.publish(temp)

            if self.ros_util.auto_function_command == 1:
                af.auto_drive_location(self.world_state, self.ros_util)
            elif self.ros_util.auto_function_command == 2:
                af.auto_dig(self.world_state, self.ros_util, 10)
            elif self.ros_util.auto_function_command == 4:
                af.auto_dump(self.world_state, self.ros_util, 4)
            elif self.ros_util.auto_function_command == 8:
                af.auto_dock(self.world_state, self.ros_util)
            elif self.ros_util.auto_function_command == 16:
                self.full_autonomy(self.world_state, self.ros_util)
            elif self.ros_util.auto_function_command == 20:
                self.test_function(self.world_state, self.ros_util)
            else:
                self.world_state.logger.info(
                    'Invalid auto-function request: {}'.format(self.ros_util.auto_function_command)
                )
        
            #self.ros_util.auto_function_command = 0
            #self.ros_util.publish_actions(uf.actions)
            temp.data = False
            self.ros_util.control_pub.publish(temp)
     


def main(args=None):
    try:
        rclpy.init(args=args)
        standard_controller_args = sys.argv[1:10]
        controller = StandardRoverController(topics=topics, args=standard_controller_args)

        controller.get_logger().info(
            'Autonomous Control Initaliazed!'
        )

        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    