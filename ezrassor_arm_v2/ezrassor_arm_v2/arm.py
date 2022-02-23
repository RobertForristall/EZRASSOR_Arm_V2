from typing import List

MOVE_GROUP_ARM: str = "moveit_arm_controller"
MOVE_GROUP_GRIPPER: str = "gripper_controller"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "ezrassor_") -> List[str]:
    return [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
    ]


def base_link_name(prefix: str = "ezrassor_") -> str:
    return "base_link"


def end_effector_name(prefix: str = "ezrassor_") -> str:
    return "link6"


def gripper_joint_names(prefix: str = "panda_") -> List[str]:
    return [
        "grabber_joint1",
        "grabber_joint2",
    ]

def home_pose() -> List[int]:
    return [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    ]

def prep_pose() -> List[int]:
    return [
        2.5167,
        -1.3500,
        1.1900,
        0.1562,
        -0.1709
    ]

def first_pickup() -> List[int]:
    return [
        2.5861,
        -1.0800,
        1.7050,
        -0.6300,
        -0.1709
    ]

def post_pickup() -> List[int]:
    return [
        2.5861,
        -0.9200,
        0.9300,
        0.0000,
        0.0000
    ]