import rclpy
import sys
import pynput
from queue import Queue
import geometry_msgs.msg
import std_msgs.msg
import threading

# Topics
front_drums_instructions_topic = 'front_drum_instructions'
back_drums_instructions_topic = 'back_drum_instructions'
front_arms_instructions_topic = 'front_arm_instructions'
back_arms_instructions_topic = 'back_arm_instructions'
wheels_instructions_topic = 'wheels_instructions'
gripper_instructions_topic = 'gripper_instructions'


