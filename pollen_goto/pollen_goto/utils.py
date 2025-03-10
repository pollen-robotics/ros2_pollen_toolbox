from enum import Enum
from typing import Callable, Optional, Tuple

import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion


def decompose_pose(pose: PoseStamped) -> Tuple[Quaternion, npt.NDArray[np.float64]]:
    translation = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

    rotation = Quaternion(
        w=pose.pose.orientation.w, x=pose.pose.orientation.x, y=pose.pose.orientation.y, z=pose.pose.orientation.z
    )
    return rotation, translation


def recompose_pose(rotation: Quaternion, translation: npt.NDArray[np.float64]) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = translation[0]
    pose.pose.position.y = translation[1]
    pose.pose.position.z = translation[2]
    pose.pose.orientation.x = rotation.x
    pose.pose.orientation.y = rotation.y
    pose.pose.orientation.z = rotation.z
    pose.pose.orientation.w = rotation.w
    return pose


def get_normal_vector(vector: npt.NDArray[np.float64], arc_direction: str) -> Optional[npt.NDArray[np.float64]]:
    """Calculate a normal vector to a given vector based on a specified direction.

    Args:
        vector: A vector [x, y, z] in 3D space.
        arc_direction: The desired direction for the normal vector. It can be one of the following options:
            'above', 'below', 'front', 'back', 'right', or 'left'.

    Returns:
        The normal vector [x, y, z] to the given vector in the specified direction. Returns `None` if the
        normal vector cannot be computed or if the vector is in the requested arc_direction.

    Raises:
        ValueError: If the arc_direction is not one of 'above', 'below', 'front', 'back', 'right', or 'left'.
    """
    match arc_direction:
        case "above":
            if abs(vector[0]) < 0.001 and abs(vector[1]) < 0.001:
                return None
            normal = np.cross(vector, [0, 0, -1])
        case "below":
            if abs(vector[0]) < 0.001 and abs(vector[1]) < 0.001:
                return None
            normal = np.cross(vector, [0, 0, 1])
        case "left":
            if abs(vector[0]) < 0.001 and abs(vector[2]) < 0.001:
                return None
            normal = np.cross(vector, [0, -1, 0])
        case "right":
            if abs(vector[0]) < 0.001 and abs(vector[2]) < 0.001:
                return None
            normal = np.cross(vector, [0, 1, 0])
        case "front":
            if abs(vector[1]) < 0.001 and abs(vector[2]) < 0.001:
                return None
            normal = np.cross(vector, [-1, 0, 0])
        case "back":
            if abs(vector[1]) < 0.001 and abs(vector[2]) < 0.001:
                return None
            normal = np.cross(vector, [1, 0, 0])
        case _:
            raise ValueError(
                f"arc_direction '{arc_direction}' not supported! Should be one of: "
                "'above', 'below', 'front', 'back', 'right' or 'left'"
            )

    if np.linalg.norm(normal) == 0:
        # Return None if the vector is in the requested arc_direction
        return None

    normal = normal / np.linalg.norm(normal)
    return normal
