from glob import glob
from operator import truediv
from time import sleep
import rclpy
import math
from geometry_msgs.msg import (Twist, Pose, Point)
import ezrassor_arm_v2.nav_functions as nf
from random import uniform
from std_msgs.msg import (Float32, Bool, Float64MultiArray, Float64)
from ezrassor_arm_interfaces.msg import ArmCommand
from sensor_msgs.msg import JointState

scan = None

actions = {
    'movement': 'stop',
    'front_arm': 0,
    'front_drum': 0,
    'back_arm': 0,
    'back_drum': 0,
    'paver_arm': ArmCommand()
}

epsilon = 0.05

class WorldState:

    def __init__(self, node):
        self.position = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.start_position = {
            'x': 0,
            'y': 0
        }
        self.arm_angles = {
            'front': 0,
            'back': 0
        }
        self.heading = 0
        self.warning_flag = 0
        self.target_location = Point()
        self.dig_site = Point()
        self.flags = {
            'on_side': False,
            'on_back': False,
            'front_up': False,
            'back_up': False
        }
        self.battery = 100
        self.hardware_Status = True
        self.carrying_dirt = False
        self.logger = node.get_logger()
    
    def jointCallBack(self, data):

        front_index = data.name.index('arm_front_joint')
        back_index = data.name.index('arm_back_joint')

        self.arm_angles['front'] = data.position[front_index]
        self.arm_angles['back'] = data.position[back_index]

        #print('Msg Angle: {}'.format(data.position[front_index]))
        #print('Svaed Angle: {}'.format(self.arm_angles['front']))


    def odometryCallBack(self, data):
        self.position['x'] = data.pose.pose.position.x + self.start_position['x']
        self.position['y'] = data.pose.pose.position.y + self.start_position['y']

        heading = nf.quaternion_to_yaw(data.pose.pose) * 180 / math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading
    
    def simStateCallBack(self, data):

        index = 0

        namespace = rclpy.get_namespace()
        namespace = namespace[1:-1] + '::base_link'
        try:
            index = data.name.index(namespace)
        except Exception:
            self.logger.info('Failed to get index, Skipping...')

        self.position['x'] = data.pose[index].position.x
        self.position['y'] = data.pose[index].position.y

        heading = nf.quaternion_to_yaw(data.pose[index]) * 180 / math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading

    def imuCallBack(self, data):

        if abs(data.linear_acceleration.y) > 9:
            self.flags['on_side'] = True
        else:
            self.flags['on_side'] = False

    def get_arm_force(self):
        front_arm_force = {
            self.state_flags['force_front_arm'] + 0.2 + uniform(-0.2, 0.2)
        }
        back_arm_force = {
            self.state_flags['force_back_arm'] + 0.2 + uniform(-0.2, 0.2)
        }
        return front_arm_force, back_arm_force

    def inital_spawn(self, x, y):
        self.start_position['x'] = x
        self.start_position['y'] = y

class ROSUtility:

    def __init__(
        self,
        node,
        rover_model,
        movement_topic,
        front_arm_topic,
        back_arm_topic,
        front_drum_topic,
        back_drum_topic,
        paver_arm_topic,
        max_linear_velocity,
        max_angular_velocity,
        obstacle_threshold,
        obstacle_buffer,
        move_increment
    ):

        queue_size = 10

        self.movement_pub = node.create_publisher(
            Twist, movement_topic, queue_size
        )

        self.back_arm_pub = node.create_publisher(
            Float64, back_arm_topic, queue_size
        )

        self.back_drum_pub = node.create_publisher(
            Float64, back_drum_topic, queue_size
        )

        if (rover_model == 'arm'):
            self.paver_arm_pub = node.create_publisher(
                ArmCommand, paver_arm_topic, queue_size
            )
        else:
            self.front_arm_pub = node.create_publisher(
                Float64, front_arm_topic, queue_size
            )
            self.front_drum_pub = node.create_publisher(
                Float64, front_drum_topic, queue_size
            )  

        self.control_pub = node.create_publisher(
            Bool, 'secondary_override_toggle', queue_size
        )

        self.arms_up_pub = node.create_publisher(
            Bool, 'arms_up', queue_size
        )

        self.rate = node.create_rate(45)

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self.auto_function_command = 0

        self.threshold = 0.5

        self.obstacle_threshold = obstacle_threshold
        self.obstacle_buffer = obstacle_buffer
        self.move_increment = move_increment

        self.rover_model = rover_model
    
    def publish_actions(self, actions):

        twist_message = Twist()

        if actions['movement'] == 'forward':
            twist_message.linear.x = self.max_linear_velocity
        elif actions['movement'] == 'reverse':
            twist_message.linear.x = -self.max_linear_velocity
        elif actions['movement'] == 'left':
            twist_message.linear.z = self.max_angular_velocity
        elif actions['movement'] == 'right':
            twist_message.linear.z = -self.max_angular_velocity
        else:
            pass

        self.movement_pub.publish(twist_message)

        back_arm_msg = Float64()
        back_arm_msg.data = float(actions['back_arm'])
        self.back_arm_pub.publish(back_arm_msg)

        back_drum_msg = Float64()
        back_drum_msg.data = float(actions['back_drum'])
        self.back_drum_pub.publish(back_drum_msg)

        if self.rover_model == 'arm':
            self.paver_arm_pub.publish(actions['paver_arm'])
        else:
            front_arm_msg = Float64()
            front_arm_msg.data = float(actions['front_arm'])
            self.front_arm_pub.publish(front_arm_msg)

            front_drum_msg = Float64()
            front_drum_msg.data = float(actions['front_drum'])
            self.front_drum_pub.publish(front_drum_msg)
        
    def autoCommandCallBack(self, data):
        self.auto_function_command = data.data
        
def on_scan_update(new_scan):
    scan = new_scan

def set_front_arm_angle(world_state, ros_util, target_angle):

    msg = Bool()

    if target_angle > world_state.arm_angles['front']:
        while target_angle > world_state.arm_angles['front']:
            print('Target: {}'.format(target_angle))
            print('Current: {}'.format(world_state.arm_angles['front']))
            actions.update({'front_arm': 1.0})
            ros_util.publish_actions(actions)
        msg.data = True
        ros_util.arms_up_pub.publish(msg)
    else:
        while target_angle < world_state.arm_angles['front']:
            actions.update({'front_arm': -1.0})
            ros_util.publish_actions(actions)
        msg.data = False
        ros_util.arms_up_pub.publish(msg)
    
    actions.update({'front_arm': 0.0})
    ros_util.publish_actions(actions)

def set_back_arm_angle(world_state, ros_util, target_angle):

    msg = Bool()

    if target_angle > world_state.arm_angles['back']:
        while target_angle > world_state.arm_angles['back']:
            actions.update({'back_arm': 1.0})
            ros_util.publish_actions(actions)
        msg.data = True
        ros_util.arms_up_pub.publish(msg)
    else:
        while target_angle < world_state.arm_angles['back']:
            actions.update({'back_arm': -1.0})
            ros_util.publish_actions(actions)
        msg.data = True
        ros_util.arms_up_pub.publish(msg)
    
    actions.update({'back_arm': 0.0})
    ros_util.publish_actions(actions)

def set_front_arm_angle_temp(world_state, ros_util, target_angle):
    arm_up_msg = Bool()
    front_arm_msg = Float64()

    if target_angle > world_state.arm_angles['front']:
        front_arm_msg.data = 1.0
        for i in range(5000):
            ros_util.front_arm_pub.publish(front_arm_msg)
        arm_up_msg.data = True
        ros_util.arms_up_pub.publish(arm_up_msg)
    else:
        front_arm_msg.data = -1.0
        for i in range(5000):
            ros_util.front_arm_pub.publish(front_arm_msg)
        arm_up_msg.data = False
        ros_util.arms_up_pub.publish(arm_up_msg)
    
    front_arm_msg.data = 0.0
    ros_util.front_arm_pub.publish(front_arm_msg)

def set_back_arm_angle_temp(world_state, ros_util, target_angle):
    arm_up_msg = Bool()
    back_arm_msg = Float64()

    if target_angle > world_state.arm_angles['back']:
        back_arm_msg.data = 1.0
        for i in range(5000):
            ros_util.back_arm_pub.publish(back_arm_msg)
        arm_up_msg.data = True
        ros_util.arms_up_pub.publish(arm_up_msg)
    else:
        back_arm_msg.data = -1.0
        for i in range(5000):
            ros_util.back_arm_pub.publish(back_arm_msg)
        arm_up_msg.data = False
        ros_util.arms_up_pub.publish(arm_up_msg)
    
    back_arm_msg.data = 0.0
    ros_util.back_arm_pub.publish(back_arm_msg)


def self_check(world_state, ros_util):

    if (
        ros_util.auto_function_command == 32
        or ros_util.auto_function_command == 0
    ):
        world_state.logger.info(
            'Canceling auto function commands...'
        )
        ros_util.publish_actions(actions)
        msg = Bool()
        msg.data = False
        ros_util.control_pub.publish(msg)
        return -1
    
    if world_state.battery < 10:
        world_state.logger.info(
            'Low battery! Rover must charge ASAP or it will halt!' 
        )
        world_state.target_location.position.x = 0
        world_state.target_location.position.y = 0
        return 3
    
    return 1

def turn(new_heading, direction, world_state, ros_util):

    twist_msg = Twist()

    angle_dist = abs((new_heading - world_state.heading + 180) % 360 - 180)
    angle_traveled = 0

    world_state.logger.info(
        'Starting turn loop...'
    )

    while angle_traveled < angle_dist - 2:

        if self_check(world_state, ros_util) != 1:
            world_state.logger.info(
                'Status check failed.'
            )
            return
        
        old_heading = world_state.heading

        turn_velocity = ros_util.max_angular_velocity * math.sin(
            (angle_traveled / math.pi) * (10 / angle_dist)
        )

        turn_velocity = max(turn_velocity, ros_util.max_angular_velocity / 10)

        if direction == 'right':
            turn_velocity *= -1
        
        twist_msg.angular.z = float(turn_velocity)
        ros_util.movement_pub.publish(twist_msg)

        #ros_util.rate.sleep()

        angle_traveled += abs(
            (world_state.heading - old_heading + 180) % 360 - 180
        )
    
    twist_msg.angular.z = 0.0
    ros_util.movement_pub.publish(twist_msg)

def move(dist, world_state, ros_util, direction="forward"):
    old_x = world_state.position['x']
    old_y = world_state.position['y']
    dist_traveled = 0
    dist_to_goal = math.sqrt(
        (world_state.target_location.x - old_x) ** 2
        + (world_state.target_location.y - old_y) ** 2
    )

    move_dist = min(dist, dist_to_goal)

    while dist_traveled < move_dist:
        if self_check(world_state, ros_util) != 1:
            world_state.logger.info(
                'Status check failed'
            )
            return
        
        if not nf.angle_is_safe(
            0,
            ros_util.obstacle_threshold / 2.0,
            ros_util.obstacle_buffer,
            scan,
            ros_util.obstacle_threshold
        ):
            world_state.logger.info(
                'Obstacle too close! Stopping!'
            )
            ros_util.publish_actions(actions)
            break
    
    move_velocity = ros_util.max_linear_velocity * math.sin(
        (dist_traveled / math.pi) * (10 / move_dist)
    )

    move_velocity = max(move_velocity, ros_util.max_linear_velocity / 10)

    if direction == "backward":
        move_velocity *= -1
    
    twist_msg = Twist()
    twist_msg.linear.x = move_velocity
    ros_util.movement_pub.publish(twist_msg)

    ros_util.rate.sleep()

    dist_traveled = math.sqrt(
        (world_state.position['x'] - old_x) ** 2
        + (world_state.position['y'] - old_y)**2
    )

def reverse_turn(world_state, ros_util):

    while world_state.warning_flag == 3:
        actions.update({'movement': 'reverse'})
        ros_util.publish_actions(actions)
        ros_util.rate.sleep()
    
    actions.update({'movement': 'stop'})
    ros_util.publish_actions(actions)

    new_heading = (world_state.heading + 60) % 360

    while (new_heading - 1) < world_state.heading < (new_heading + 1):
        actions.update({'movement': 'left'})
        ros_util.publish_actions(actions)
    
    actions.update({'movement': 'stop'})
    ros_util.publish_actions(actions)

def dodge_left(world_state, ros_util):

    start_x = world_state.position['x']
    start_y = world_state.position['y']

    threshold = 0

    while world_state.warning_flag != 0 or (threshold < 25):
        if world_state.warning_flag == 0:
            threshold += 1
        actions.update({'movement': 'left'})
        ros_util.publish_actions(actions)
        ros_util.rate.sleep()
    
    while(
        nf.euclidean_distance(
            start_x, world_state.position['x'], start_y, world_state.position['y']
        ) < 2
    ):
        actions.update({'movement': 'forward'})
        ros_util.publish_actions(actions)
        ros_util.rate.sleep()
    
    actions.update({'movement': 'stop'})
    ros_util.publish_actions(actions)

def dodge_right(world_state, ros_util):

    start_x = world_state.position['x']
    start_y = world_state.position['y']

    threshold = 0

    while world_state.warning_flag != 0 or (threshold < 25):
        if world_state.warning_flag == 0:
            threshold += 1
        actions.update({'movement': 'right'})
        ros_util.publish_actions(actions)
        ros_util.rate.sleep()
    
    while(
        nf.euclidean_distance(
            start_x, world_state.position['x'], start_y, world_state.position['y']
        ) < 2
    ):
        actions.update({'movement': 'forward'})
        ros_util.publish_actions(actions)
        ros_util.rate.sleep()
    
    actions.update({'movement': 'stop'})
    ros_util.publish_actions(actions)

def self_right_from_side(world_state, ros_util):

    world_state.logger.info(
        'Staring auto self-right...'
    )

    while world_state.flags['on_side'] is not False:
        actions.update({'front_arm': 1})
        ros_util.publish_actions(actions)
        actions.update({'front_arm': 0, 'back_arm': 1})
        ros_util.publish_actions(actions)

    actions.update({'front_arm': 0, 'back_arm': 1})
    ros_util.publish_actions(actions)


def get_turn_angle(world_state, ros_util):

    best_angle = nf.get_best_angle(
        world_state, ros_util.obstacle_buffer, scan, ros_util.obstacle_threshold
    )

    while True:

        if best_angle is None:
            
            switch_direction = -1
            wedge_dist = 0
            wedge_size = (scan.angle_max - scan.angle_min) / 2.0
            world_state.logger.info(
                'There is nowhere to go in the current wedge.'
                + 'Turning to the adjacent wedge.'
            )

            while best_angle is None:
                if self_check(world_state, ros_util) != 1:
                    world_state.logger.info(
                        'Status Check Failed!'
                    )
                    return
                
                set_front_arm_angle(world_state, ros_util, 1.3)
                set_back_arm_angle(world_state, ros_util, 1.3)

                switch_direction *= -1
                wedge_dist += 1

                if switch_direction < 0:
                    direction = 'left'
                else:
                    direction = 'right'
                
                turn (
                    nf.rel_to_abs(world_state.heading, wedge_size * wedge_dist),
                    direction,
                    world_state,
                    ros_util
                )

                ros_util.rate.sleep()

                world_state.logger.info(
                    'Currently at wedge W{}'.format(wedge_dist-1)
                )
                best_angle = nf.get_best_angle(
                    world_state,
                    ros_util.obstacle_buffer,
                    scan,
                    ros_util.obstacle_threshold,
                )
        wedge_size = (scan.angle_max - scan.angle_min) / 20.0
        buffer_angle = math.atan(
            ros_util.obstacle_buffer / ros_util.obstacle_threshold
        )
        min_angle = scan.angle_min + buffer_angle
        max_angle = scan.angle_max - buffer_angle
        best_index = int((best_angle - scan.angle_min) / scan.angle_increment)
        min_index = int((min_angle - scan.angle_min) / scan.angle_increment)
        max_index = int((max_angle - scan.angle_min) / scan.angle_increment)

        while best_index <= min_index or best_index >= max_index:
            if best_angle < 0:
                direction = 'right'
            else:
                direction = 'left'
            
            turn(
                nf.rel_to_abs(world_state.heading, wedge_size),
                direction,
                world_state,
                ros_util,
            )

            ros_util.rate.sleep()

            best_angle = nf.get_best_angle(
                world_state,
                ros_util.obstacle_buffer,
                scan,
                ros_util.obstacle_threshold,
            )

            if best_angle is None:
                break
            
            best_index = int(
                (best_angle - scan.angle_min) / scan.angle_increment
            )
        
        if best_angle is not None:
            return best_angle
                

def is_force_target(world_state, target_force):

    front_force = (
        target_force - epsilon
        < world_state.state_flags['force_front_arm']
        < target_force + epsilon
    )

    back_force = (
        target_force - epsilon
        < world_state.state_flags['force_back_arm']
        < target_force + epsilon
    )

    return front_force and back_force

def set_target_force(world_state, ros_util, target_force, t):

    while not is_force_target(world_state, target_force):

        msg = 0b0
        if (
            not target_force - epsilon
            < world_state.state_flags['force_front_arm']
            < target_force + epsilon
        ):
            if target_force > world_state.state_flags['force_front_arm']:
                msg += ros_util.commands['front_arm_up']
            elif target_force < world_state.state_flags['force_front_arm']:
                msg += ros_util.commands['front_arm_down']

        if (
            not target_force - epsilon
            < world_state.state_flags['force_back_arm']
            < target_force + epsilon
        ):
            if target_force > world_state.state_flags['force_back_arm']:
                msg += ros_util.commands['back_arm_up']
            elif target_force < world_state.state_flags['force_back_arm']:
                msg += ros_util.commands['back_arm_down']
        
        msg += ros_util.commands['front_dig']
        msg += ros_util.commands['back_dig']
        msg += ros_util.commands['forward']
        ros_util.command_pub.publish(msg)
        ros_util.rate.sleep()
        t += 1
    
    ros_util.command_pub.publish(ros_util.commands['null'])
    return t




