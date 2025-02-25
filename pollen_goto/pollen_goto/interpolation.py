"""Trajectory interpolation utility module.

Provides two main interpolation methods:
- linear
- minimum jerk
"""

from enum import Enum
from typing import Callable, Optional, Tuple

import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion

from .utils import decompose_pose, get_normal_vector, recompose_pose

InterpolationFunc = Callable[[float], np.ndarray | PoseStamped]

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


def cartesian_linear(
    starting_pose: np.ndarray,
    goal_pose: PoseStamped,
    duration: float,
    arc_direction: str,
    secondary_radius: float,
) -> InterpolationFunc:
    """Compute the linear interpolation function from starting pose to goal pose."""
    q_origin, trans_origin = decompose_pose(starting_pose)
    q_target, trans_target = decompose_pose(goal_pose)

    def f(t: float) -> np.ndarray:
        if t > duration:
            return goal_pose
        trans_interpolated = trans_origin + (trans_target - trans_origin) * t / duration
        q_interpolated = Quaternion.slerp(q_origin, q_target, t / duration)
        pose = recompose_pose(q_interpolated, trans_interpolated)
        return pose

    return f


def cartesian_minimum_jerk(
    starting_pose: np.ndarray,
    goal_pose: PoseStamped,
    duration: float,
    arc_direction: str,
    secondary_radius: float,
) -> InterpolationFunc:
    """Compute the mimimum jerk interpolation function from starting pose to goal pose."""
    q_origin, trans_origin = decompose_pose(starting_pose)
    q_target, trans_target = decompose_pose(goal_pose)

    a0 = trans_origin
    a1 = np.zeros(3)
    a2 = np.zeros(3)

    d1, d2, d3, d4, d5 = [duration**i for i in range(1, 6)]

    A = np.array(((d3, d4, d5), (3 * d2, 4 * d3, 5 * d4), (6 * d1, 12 * d2, 20 * d3)))
    B = np.array(
        (
            trans_target - a0 - (a1 * d1) - (a2 * d2),
            np.zeros(3) - a1 - (2 * a2 * d1),
            np.zeros(3) - (2 * a2),
        )
    )
    X = np.linalg.solve(A, B)

    coeffs = [a0, a1, a2, X[0], X[1], X[2]]

    def f(t: float) -> np.ndarray:
        if t > duration:
            return goal_pose
        trans_interpolated = np.sum([c * t**i for i, c in enumerate(coeffs)], axis=0)
        q_interpolated = Quaternion.slerp(q_origin, q_target, t / duration)
        pose = recompose_pose(q_interpolated, trans_interpolated)
        return pose

    return f


def cartesian_elliptical(
    starting_pose: np.ndarray,
    goal_pose: PoseStamped,
    duration: float,
    arc_direction: str,
    secondary_radius: float,
) -> InterpolationFunc:
    """Compute the elliptical interpolation function from starting pose to goal pose."""

    print("Elliptical interpolation")
    q_origin, trans_origin = decompose_pose(starting_pose)
    q_target, trans_target = decompose_pose(goal_pose)

    vector_target_origin = trans_target - trans_origin

    center = (trans_origin + trans_target) / 2
    radius = float(np.linalg.norm(vector_target_origin) / 2)

    vector_origin_center = trans_origin - center
    vector_target_center = trans_target - center

    if secondary_radius < 0:
        secondary_radius = radius

    normal = get_normal_vector(vector=vector_target_origin, arc_direction=arc_direction)

    if np.isclose(radius, 0, atol=1e-03) or normal is None:
        return cartesian_linear(starting_pose, goal_pose, duration)

    cos_angle = np.dot(vector_origin_center, vector_target_center) / (
        np.linalg.norm(vector_origin_center) * np.linalg.norm(vector_target_center)
    )
    angle = np.arccos(np.clip(cos_angle, -1, 1))

    def f(t: float) -> np.ndarray:
        if t > duration:
            return goal_pose

        theta = t / duration * angle

        # Rotation of origin_vector around the circle center in the plan defined by 'normal'
        q1 = Quaternion(axis=normal, angle=theta)
        rotation_matrix = q1.rotation_matrix
        # Interpolated point in plan
        trans_interpolated = np.dot(rotation_matrix, vector_origin_center)
        # Find the major and minor axes in the plane of the ellipse
        major_axis = vector_target_origin / np.linalg.norm(vector_target_origin)
        minor_axis = np.cross(normal, major_axis)
        minor_axis = minor_axis / np.linalg.norm(minor_axis)

        # Project the interpolated point onto the major and minor axes
        major_component = np.dot(trans_interpolated, major_axis)
        minor_component = np.dot(trans_interpolated, minor_axis)

        # Adjust the ellipse using the secondary radius
        adjusted_trans = (
            major_component * major_axis +
            (minor_component * (secondary_radius / radius)) * minor_axis
        )
        trans_interpolated = adjusted_trans + center

        q_interpolated = Quaternion.slerp(q_origin, q_target, t / duration)

        pose = recompose_pose(q_interpolated, trans_interpolated)
        return pose

    return f


class JointSpaceInterpolationMode(Enum):
    """Inteprolation Mode enumeration."""

    LINEAR: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = linear
    MINIMUM_JERK: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = minimum_jerk


class CartesianSpaceInterpolationMode(Enum):
    """Inteprolation Mode enumeration."""

    LINEAR: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = cartesian_linear
    MINIMUM_JERK: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = cartesian_minimum_jerk
    ELLIPTICAL: Callable[[np.ndarray, np.ndarray, float], InterpolationFunc] = cartesian_elliptical
