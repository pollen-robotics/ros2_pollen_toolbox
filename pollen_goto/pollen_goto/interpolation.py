"""Trajectory interpolation utility module.

Provides two main interpolation methods:
- linear
- minimum jerk
"""

from enum import Enum
from typing import Callable, Optional
from pyquaternion import Quaternion
from geometry_msgs.msg import Pose

import numpy as np

InterpolationFunc = Callable[[float], np.ndarray | Pose]

# Note: for these functions to behave correctly, motor positions should be monotonic. Meaning no "179->180->-180" transitions.


def linear(
    starting_position: np.ndarray,
    goal_position: np.ndarray,
    duration: float,
    starting_velocity: Optional[np.ndarray] = None,
    starting_acceleration: Optional[np.ndarray] = None,
    final_velocity: Optional[np.ndarray] = None,
    final_acceleration: Optional[np.ndarray] = None,
) -> InterpolationFunc:
    """Compute the linear interpolation function from starting position to goal position."""

    def f(t: float) -> np.ndarray:
        if t > duration:
            return goal_position
        return starting_position + (goal_position - starting_position) * t / duration

    return f


def minimum_jerk(
    starting_position: np.ndarray,
    goal_position: np.ndarray,
    duration: float,
    starting_velocity: Optional[np.ndarray] = None,
    starting_acceleration: Optional[np.ndarray] = None,
    final_velocity: Optional[np.ndarray] = None,
    final_acceleration: Optional[np.ndarray] = None,
) -> InterpolationFunc:
    """Compute the mimimum jerk interpolation function from starting position to goal position."""
    if starting_velocity is None:
        starting_velocity = np.zeros(starting_position.shape)
    if starting_acceleration is None:
        starting_acceleration = np.zeros(starting_position.shape)
    if final_velocity is None:
        final_velocity = np.zeros(goal_position.shape)
    if final_acceleration is None:
        final_acceleration = np.zeros(goal_position.shape)

    a0 = starting_position
    a1 = starting_velocity
    a2 = starting_acceleration / 2

    d1, d2, d3, d4, d5 = [duration**i for i in range(1, 6)]

    A = np.array(((d3, d4, d5), (3 * d2, 4 * d3, 5 * d4), (6 * d1, 12 * d2, 20 * d3)))
    B = np.array(
        (
            goal_position - a0 - (a1 * d1) - (a2 * d2),
            final_velocity - a1 - (2 * a2 * d1),
            final_acceleration - (2 * a2),
        )
    )
    X = np.linalg.solve(A, B)

    coeffs = [a0, a1, a2, X[0], X[1], X[2]]

    def f(t: float) -> np.ndarray:
        if t > duration:
            return goal_position
        return np.sum([c * t**i for i, c in enumerate(coeffs)], axis=0)

    return f


def cartesian_linear(starting_pose: np.ndarray, goal_pose: Pose, duration: float) -> InterpolationFunc:
    """Compute the linear interpolation function from starting pose to goal pose."""

    def decompose_pose(pose: Pose) -> Tuple[Quaternion, npt.NDArray[np.float64]]:
        translation = [pose.position.x, pose.position.y, pose.position.z]

        rotation = Quaternion(w=pose.orientation.w, x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z, atol=1e-05, rtol=1e-05)
        return rotation, translation

    def recompose_pose(rotation: Quaternion, translation: npt.NDArray[np.float64]) -> Pose:
        pose = Pose()
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        pose.orientation.x = rotation.x
        pose.orientation.y = rotation.y
        pose.orientation.z = rotation.z
        pose.orientation.w = rotation.w
        return pose

    q_start, trans_start = decompose_pose(starting_pose)
    q_stop, trans_stop = decompose_pose(goal_pose)

    def f(t: float) -> np.ndarray:
        if t > duration:
            return goal_pose
        trans_interpolated = trans_start + (trans_stop - trans_start) * t / duration
        q_interpolated = Quaternion.slerp(q_start, q_stop, t / duration)
        pose = recompose_pose(q_interpolated, trans_interpolated)
        return pose

    return f


def cartesian_minimum_jerk(starting_pose: np.ndarray, goal_pose: np.ndarray, duration: float) -> InterpolationFunc:
    """Compute the mimimum jerk interpolation function from starting pose to goal pose."""
    pass


class JointSpaceInterpolationMode(Enum):
    """Inteprolation Mode enumeration."""

    LINEAR: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = linear
    MINIMUM_JERK: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = (
        minimum_jerk
    )


class CartesianSpaceInterpolationMode(Enum):
    """Inteprolation Mode enumeration."""

    LINEAR: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = cartesian_linear
    MINIMUM_JERK: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = (
        cartesian_minimum_jerk
    )
