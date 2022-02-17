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