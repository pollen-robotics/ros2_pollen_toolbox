"""Example of how to draw a square with Reachy's right arm."""

import logging
import time
from typing import Sequence

import numpy as np
import numpy.typing as npt

from reachy2_sdk import ReachySDK

from reachy2_sdk.parts.joints_based_part import JointsBasedPart



def build_pose_matrix(x: float, y: float, z: float) -> npt.NDArray[np.float64]:
    """Build a 4x4 pose matrix for a given position in 3D space, with the effector at a fixed orientation.

    Args:
        x: The x-coordinate of the position.
        y: The y-coordinate of the position.
        z: The z-coordinate of the position.

    Returns:
        A 4x4 NumPy array representing the pose matrix.
    """
    # The effector is always at the same orientation in the world frame
    return np.array(
        [
            [0, 0, -1, x],
            [0, 1, 0, y],
            [1, 0, 0, z],
            [0, 0, 0, 1],
        ]
    )

def compute_stats(durations: Sequence[float]) -> dict[str, float]:
    """Return basic descriptive statistics for a sequence of durations."""
    duration_array = np.array(durations, dtype=np.float64)
    avg_duration = float(duration_array.mean())
    max_duration = float(duration_array.max())
    return {
        "avg_duration": avg_duration,
        "avg_frequency": float(1.0 / avg_duration) if avg_duration > 0 else float("inf"),
        "variance": float(duration_array.var()),
        "max_duration": max_duration,
        "worst_frequency": float(1.0 / max_duration) if max_duration > 0 else float("inf"),
    }


def log_stats(label: str, durations: Sequence[float]) -> None:
    stats = compute_stats(durations)
    logging.info(
        f"{label} IK stats over {len(durations)} runs"
    )
    logging.info(f"  Avg duration: {stats['avg_duration']:.6f} s")
    logging.info(f"  Avg frequency: {stats['avg_frequency']:.2f} Hz")
    logging.info(f"  Variance: {stats['variance']:.8f} s^2")
    logging.info(f"  Max duration (slowest): {stats['max_duration']:.6f} s")
    logging.info(f"  Worst frequency: {stats['worst_frequency']:.2f} Hz")


def benchmark_inverse_kinematics(
    part: JointsBasedPart, target_pose: npt.NDArray[np.float64], runs: int = 50
) -> None:
    """Call IK repeatedly and log timing statistics."""
    durations: list[float] = []

    for _ in range(runs):
        start = time.perf_counter()
        part.inverse_kinematics(target_pose)
        durations.append(time.perf_counter() - start)

    part_name = getattr(part, "name", part.__class__.__name__)
    log_stats(part_name, durations)


def benchmark_dual_inverse_kinematics(
    right_part: JointsBasedPart,
    right_pose: npt.NDArray[np.float64],
    left_part: JointsBasedPart,
    left_pose: npt.NDArray[np.float64],
    runs: int = 50,
) -> None:
    """Call both IK solvers sequentially per iteration and log combined timings."""
    durations: list[float] = []

    for _ in range(runs):
        start = time.perf_counter()
        right_part.inverse_kinematics(right_pose)
        left_part.inverse_kinematics(left_pose)
        durations.append(time.perf_counter() - start)

    right_name = getattr(right_part, "name", right_part.__class__.__name__)
    left_name = getattr(left_part, "name", left_part.__class__.__name__)
    log_stats(f"{right_name} + {left_name}", durations)


if __name__ == "__main__":
    print("Reachy SDK example: draw square")

    logging.basicConfig(level=logging.INFO)
    reachy = ReachySDK(host="localhost")

    if not reachy.is_connected:
        exit("Reachy is not connected.")

    print("Turning on Reachy")
    reachy.turn_on()

    time.sleep(0.2)
    try:
        runs = 200
        r_target_pose = build_pose_matrix(0.4, -0.5, 0)
        l_target_pose = build_pose_matrix(0.4, 0.5, 0)

        print("\nOnly right arm IK benchmark:")
        benchmark_inverse_kinematics(reachy.r_arm, r_target_pose, runs)
        print("\nOnly left arm IK benchmark:")
        benchmark_inverse_kinematics(reachy.l_arm, l_target_pose, runs)
        print("\nright arm + left arm IK benchmark:")
        benchmark_dual_inverse_kinematics(
            reachy.r_arm, r_target_pose, reachy.l_arm, l_target_pose, runs
        )

    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback

        traceback.print_exc()
