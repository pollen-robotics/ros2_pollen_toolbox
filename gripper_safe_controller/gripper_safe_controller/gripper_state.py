import time
from collections import deque
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from scipy import interpolate

## Servomotor related parameters
P_GAIN = 4.0
# max_torque set in the servo (max value is 100). This is a firmware configuration.
DYNAMIXEL_MAX_TORQUE = 50
# With a P of 4, any error greater than SATURATION_ERROR will put the PWM at 100%
SATURATION_ERROR = np.deg2rad(5.7)
# This is the maximum continuous error that the servo can apply without overheating (>65° in a room at 20° and a P of 4).
MAX_SAFE_ERROR = np.deg2rad(1.0)
# TEMP setting a huge value here effectively disables the not so smart gripper
MAX_COLLISION_ERROR = np.deg2rad(1000.0)  # MAX_SAFE_ERROR
# 378 deg/s is the no load speed at 12V for the MX-64. However, the actual speed is lower due to the load.
# This was directly measured as the average cruise speed during a closing motion
MAX_SPEED = np.deg2rad(195)

# This is the natural tracking error of the gripper during its cruise speed
DYNAMIC_ERROR = 0.140  # 0.099
# Maximum temperature is fixed at 65°
# Deprecated for now. This algorithm used to change the PID values when a collision was detected.
# P_SAFE_CLOSE = 3.0  # 3.0
# P_DIRECT_CONTROL = 5.0
# MX28_TO_MX64_RATIO = 2.5 / 6.0


## Control algorithm related parameters
# Maximum error in rads that will be requested to the servo by this control algorithm.
# This directly controls the maximum force that will be applied by the gripper. To get the actual force in N, use get_error_to_apply_force
MAX_APPLIED_ERROR = MAX_SAFE_ERROR * 0.5
HISTORY_LENGTH = 10
# After a change in goal position, there is no possible collision detection during the first SKIP_EARLY_DTS control cycles
# This is to avoid false positives during the acceleration phase. The gripper reaches its cruising speed after ~40ms.
SKIP_EARLY_DTS = 6  # 4
# When in collision, this parameter in rads is the minimum distance that the gripper will move before the collision is considered over
MIN_MOVING_DIST = 0.04  # 0.075
UPDATE_FREQ = 100  # Hz
# TODO this value should be read from a controller parameter instead
DT = 1 / UPDATE_FREQ
# Maximum mouvement per control cycle in rads
MAX_INC_PER_DT = MAX_SPEED * DT
# If the servo is moving faster than this, it will never be considered in collision
TOO_FAST_TO_COLLIDE = 0.75 * MAX_INC_PER_DT

## Gripper related parameters
MAX_OPENING_MM = 95.09
ANGLE_TO_PERCENT_OPENING = [
    (np.deg2rad(130), 95.09 * 100 / MAX_OPENING_MM),
    (np.deg2rad(120), 91.44 * 100 / MAX_OPENING_MM),
    (np.deg2rad(110), 86.72 * 100 / MAX_OPENING_MM),
    (np.deg2rad(100), 80.81 * 100 / MAX_OPENING_MM),
    (np.deg2rad(90), 73.79 * 100 / MAX_OPENING_MM),
    (np.deg2rad(80), 65.96 * 100 / MAX_OPENING_MM),
    (np.deg2rad(70), 57.2 * 100 / MAX_OPENING_MM),
    (np.deg2rad(60), 48.33 * 100 / MAX_OPENING_MM),
    (np.deg2rad(50), 38.76 * 100 / MAX_OPENING_MM),
    (np.deg2rad(40), 29.43 * 100 / MAX_OPENING_MM),
    (np.deg2rad(30), 20.32 * 100 / MAX_OPENING_MM),
    (np.deg2rad(20), 12.29 * 100 / MAX_OPENING_MM),
    (np.deg2rad(10), 5.34 * 100 / MAX_OPENING_MM),
    (np.deg2rad(0), 0),
]
# Sorting the array is important here
# TODO opening_to_angle and angle_to_opening work here because sorting the array for [1] is also sorting it for [0]
ANGLE_TO_PERCENT_OPENING.sort(key=lambda x: x[1])


class CollisionState(Enum):
    NO_COLLISION = 0
    ENTERING_COLLISION = 1
    STILL_COLLIDING = 2
    LEAVING_COLLISION = 3


class GripperState:
    """Represent the current gripper state."""

    def __init__(
        self,
        name: str,
        is_direct: bool,
        present_position: float,
        user_requested_goal_position: float,
        p: float = P_GAIN,
        i: float = 0.0,
        d: float = 0.0,
        logger=None,
        torque_limit=20.0,
    ) -> None:
        self.name = name
        self.is_direct = is_direct

        self.logger = logger
        self.present_position = deque([present_position], HISTORY_LENGTH)
        self.user_requested_goal_position = deque(
            [user_requested_goal_position], HISTORY_LENGTH
        )

        self.interpolated_goal_position = deque(
            [user_requested_goal_position], HISTORY_LENGTH
        )
        self.error = deque([], HISTORY_LENGTH // 2)
        self.in_collision = deque([False], HISTORY_LENGTH)

        self.safe_computed_goal_position = user_requested_goal_position

        self.elapsed_dts_since_change_of_direction = 0
        self.elapsed_dts_since_collision = 0

        self.pid = p, i, d
        self.torque_limit = torque_limit

        self.calculate_fit_and_derivative_of_opening()

    def update(
        self, new_present_position: float, new_user_requested_goal_position: float
    ):
        self.present_position.append(new_present_position)

        if self.has_changed_direction(new_user_requested_goal_position):
            self.elapsed_dts_since_change_of_direction = 0

        self.user_requested_goal_position.append(new_user_requested_goal_position)

        collision_state = self.check_collision_state()

        if collision_state == CollisionState.NO_COLLISION:
            interpolated_goal_position = self.compute_close_smart_goal_position()
            self.safe_computed_goal_position = new_user_requested_goal_position

        elif collision_state == CollisionState.ENTERING_COLLISION:
            # self.set_pid(p=P_SAFE_CLOSE, i=0.0, d=0.0)
            interpolated_goal_position = self.compute_fake_error_goal_position()
            self.safe_computed_goal_position = interpolated_goal_position

        elif collision_state == CollisionState.STILL_COLLIDING:
            interpolated_goal_position = self.compute_fake_error_goal_position()
            self.safe_computed_goal_position = interpolated_goal_position

        elif collision_state == CollisionState.LEAVING_COLLISION:
            # self.set_pid(p=P_DIRECT_CONTROL, i=0.0, d=0.0)
            interpolated_goal_position = self.compute_close_smart_goal_position()
            self.safe_computed_goal_position = new_user_requested_goal_position

        self.interpolated_goal_position.append(interpolated_goal_position)
        # Estimation of the tracking error. Here the expected speed is a linear function of the error
        raw_error = interpolated_goal_position - new_present_position
        saturated_error = np.clip(raw_error, -SATURATION_ERROR, SATURATION_ERROR)
        max_dynamic_error = DYNAMIC_ERROR if self.is_direct else -DYNAMIC_ERROR
        error = raw_error + saturated_error * max_dynamic_error / SATURATION_ERROR
        self.error.append(error)

        if self.name.startswith("r"):
            self.logger.debug(
                f"State: {collision_state}, pres_pos: {new_present_position}, interpol={interpolated_goal_position}, final_goal: {new_user_requested_goal_position}, err: {self.error[-1]}"
            )
            self.logger.debug(f"raw_error: {raw_error} better_error: {error}")
        self.in_collision.append(
            collision_state
            in (CollisionState.ENTERING_COLLISION, CollisionState.STILL_COLLIDING)
        )

        self.elapsed_dts_since_change_of_direction += 1
        self.elapsed_dts_since_collision += 1

    def check_collision_state(self) -> CollisionState:
        if not hasattr(self, "_hidden_collision_state"):
            self._hidden_collision_state = CollisionState.NO_COLLISION

        if not self.in_collision[-1] and self.entering_collision():
            self._hidden_collision_state = CollisionState.STILL_COLLIDING
            return CollisionState.ENTERING_COLLISION

        # if self.in_collision[-1] and not self.entering_collision():
        #     self._hidden_collision_state = CollisionState.NO_COLLISION
        #     self.elapsed_dts_since_collision = 0
        #     return CollisionState.LEAVING_COLLISION

        if self.in_collision[-1] and self.leaving_collision():
            self._hidden_collision_state = CollisionState.NO_COLLISION
            self.elapsed_dts_since_collision = 0
            return CollisionState.LEAVING_COLLISION

        return self._hidden_collision_state

    def entering_collision(self) -> bool:
        if self.elapsed_dts_since_change_of_direction <= SKIP_EARLY_DTS:
            if self.name.startswith("r"):
                self.logger.debug(f"STILL IN ELAPSED")
            return False

        # filtered_error = np.mean(self.error)
        filtered_error = self.error[-1]
        delta_pos = abs(self.present_position[-1] - self.present_position[-2])
        if self.name.startswith("r"):
            self.logger.debug(
                f"collision_check. name:{self.name}, error:{filtered_error:.3f}, MAX_COLLISION_ERROR:{MAX_COLLISION_ERROR:.3f}, last_inc:{delta_pos:.3f}, TOO_FAST_TO_COLLIDE:{TOO_FAST_TO_COLLIDE:.3f}"
            )
        # if it's moving fast enough, it will never be considered in collision
        if delta_pos > TOO_FAST_TO_COLLIDE:
            return False
        return (
            (
                filtered_error > MAX_COLLISION_ERROR
                and self.user_requested_goal_position[-1] > self.present_position[-1]
            )
            if self.is_direct
            else (
                filtered_error < -MAX_COLLISION_ERROR
                and self.user_requested_goal_position[-1] < self.present_position[-1]
            )
        )

    def leaving_collision(self) -> bool:
        last_req_goal_pos = self.user_requested_goal_position[-1]
        last_interp_goal_pos = self.interpolated_goal_position[-1]

        # This condition is triggered by a direct user command to release our grasp
        user_request_to_release = (
            last_req_goal_pos < last_interp_goal_pos
            if self.is_direct
            else last_req_goal_pos > last_interp_goal_pos
        )

        # This condition is triggered because we are moving again, due to either:
        #   - because the object was removed
        #   - because it was a false detection in the first place
        moving_again = (
            self.elapsed_dts_since_collision > self.present_position.maxlen
            and (
                (self.present_position[0] - self.present_position[-1])
                < -MIN_MOVING_DIST
                if self.is_direct
                else (self.present_position[0] - self.present_position[-1])
                > MIN_MOVING_DIST
            )
        )
        # self.logger.info(
        #     f"user_request_to_release={user_request_to_release}, moving_again={moving_again}"
        # )
        return user_request_to_release or moving_again

    def has_changed_direction(self, new_goal_pos: float) -> bool:
        present_position = self.present_position[-1]
        last_goal_pos = self.user_requested_goal_position[-1]

        return np.sign(last_goal_pos - present_position) != np.sign(
            new_goal_pos - present_position
        )

    def compute_close_smart_goal_position(self) -> float:
        """Outputs a goal position that is always close to the previous goal position.
        So if the user requests a goal position that is far from the current position, this method will create an interpolation based on the max speed allowed.
        """
        last_req_goal_pos = self.user_requested_goal_position[-1]
        goal_offset = MAX_INC_PER_DT * np.sign(
            last_req_goal_pos - self.present_position[-1]
        )

        next_goal_pos = self.interpolated_goal_position[-1] + goal_offset

        return (
            min(next_goal_pos, last_req_goal_pos)
            if goal_offset > 0
            else max(next_goal_pos, last_req_goal_pos)
        )

    def compute_fake_error_goal_position(self) -> float:
        """Simple method that will output a goal position at a fixed distance from the present position."""
        # Note: while this approach is simple and robust, we can easily improve it by providing a function
        # that emultes a force control by calling get_error_to_apply_force.
        model_offset = MAX_APPLIED_ERROR
        if not self.is_direct:
            model_offset = -model_offset

        return model_offset + self.present_position[-1]

    def set_pid(self, p, i, d) -> None:
        self.pid = p, i, d

    def opening_to_angle(self, opening):
        """Simple linear interpolation to link an opening (100 means fully open, 0 means fully closed) to an angle command for the gripper"""
        # Handle edge cases
        if opening <= 0:
            return ANGLE_TO_PERCENT_OPENING[0][0]
        elif opening >= 100:
            return ANGLE_TO_PERCENT_OPENING[-1][0]

        for i in range(1, len(ANGLE_TO_PERCENT_OPENING)):
            if opening == ANGLE_TO_PERCENT_OPENING[i][1]:
                return ANGLE_TO_PERCENT_OPENING[i][0]
            elif opening < ANGLE_TO_PERCENT_OPENING[i][1]:
                # Perform linear interpolation
                x0, y0 = ANGLE_TO_PERCENT_OPENING[i - 1]
                x1, y1 = ANGLE_TO_PERCENT_OPENING[i]
                return x0 + (x1 - x0) * ((opening - y0) / (y1 - y0))

        # Should never happen
        return ANGLE_TO_PERCENT_OPENING[0][0]

    def angle_to_opening(self, angle_rad):
        # Handle edge cases
        if angle_rad <= ANGLE_TO_PERCENT_OPENING[0][0]:
            return ANGLE_TO_PERCENT_OPENING[0][1]
        elif angle_rad >= ANGLE_TO_PERCENT_OPENING[-1][0]:
            return ANGLE_TO_PERCENT_OPENING[-1][1]

        for i in range(1, len(ANGLE_TO_PERCENT_OPENING)):
            prev_angle_deg, prev_percent = (
                ANGLE_TO_PERCENT_OPENING[i - 1][0],
                ANGLE_TO_PERCENT_OPENING[i - 1][1],
            )
            curr_angle_deg, curr_percent = (
                ANGLE_TO_PERCENT_OPENING[i][0],
                ANGLE_TO_PERCENT_OPENING[i][1],
            )
            if prev_angle_deg <= angle_rad < curr_angle_deg:
                # Perform linear interpolation
                interpolated_percent = prev_percent + (curr_percent - prev_percent) * (
                    (angle_rad - prev_angle_deg) / (curr_angle_deg - prev_angle_deg)
                )
                return interpolated_percent

    def get_correction_factor(self, angle):
        """This method returns a correction factor that can be used to adjust the torque applied by the motor to the force applied by the fingers.
        If the relationship between the torque and the force is linear, this correction factor should be close to 1.
        A number greater than 1 means that the force applied by the gripper is larger.
        """
        linear_approx = 100 / (
            ANGLE_TO_PERCENT_OPENING[-1][0] - ANGLE_TO_PERCENT_OPENING[0][0]
        )

        ratio = self.opening_derivative(angle) / linear_approx
        return ratio

    def get_error_to_apply_force(self, goal_position, goal_force):
        """This method returns the positional error (rads) to be applied so that the gripper applies a given force (N).
        This relationship is dependant on the position of the gripper hence the goal_position parameter.
        """
        # Detail of the measures
        # https://www.notion.so/pollen-robotics/Enhance-SmartGripper-with-the-final-version-of-Beta-s-Gripper-e7bed050f5ca4ecab42dab45bac563c5?pvs=4#e2790e854d514a469e2db615b6e14eb7
        # tl dr at angle 34° with a P of 4 and a max_torque (dynamiel parameter) of 50%, the relationship between the error and the force measured at the fingers is: force(N) = 5.51 * error (°).
        # This is a simple model that ignores friction
        error = goal_force / 5.51
        error = np.deg2rad(error)
        correction_factor = self.get_correction_factor(goal_position)
        correction_factor_at_34_deg = self.get_correction_factor(np.deg2rad(34))
        error = error * correction_factor_at_34_deg / correction_factor
        error = np.clip(error, -SATURATION_ERROR, SATURATION_ERROR)
        return error

    def calculate_fit_and_derivative_of_opening(self):
        """This is a better implementation of the angle_to_opening function, using a polynomial fit to the data.
        This allows to calculate the symbolic derivative of the opening with respect to the angle.
        This derivative is a good approximation of the relationship between the torque applied by the motor and the force applied by the fingers
        """
        # Define the angle and opening data
        angle_data = [angle for angle, _ in ANGLE_TO_PERCENT_OPENING]
        opening_data = [opening for _, opening in ANGLE_TO_PERCENT_OPENING]

        # Fit a polynomial to the angle-to-opening data
        degree = 5
        poly_coefficients = np.polyfit(angle_data, opening_data, degree)
        self.opening_fit = np.poly1d(poly_coefficients)

        # Create a symbolic polynomial
        x = sp.symbols("x")
        symbolic_poly = sum(
            [coeff * x**i for i, coeff in enumerate(poly_coefficients[::-1])]
        )

        symbolic_derivative = sp.diff(symbolic_poly, x)
        self.opening_derivative = sp.lambdify(x, symbolic_derivative, "numpy")


if __name__ == "__main__":
    gripper = GripperState("debug", True, 0, 0)
    test_values = [0, 25, 50, 75, 100]
    results = {value: gripper.opening_to_angle(value) for value in test_values}
    print(f"Testing opening_to_angle: {results}")

    test_values = [
        np.deg2rad(0),
        np.deg2rad(30),
        np.deg2rad(65),
        np.deg2rad(90),
        np.deg2rad(130),
    ]
    results = {value: gripper.angle_to_opening(value) for value in test_values}
    print(f"\nTesting angle_to_opening: {results}\n")

    for angle in range(0, 130, 10):
        print(
            f"Angle {angle:.0f}°: opening: {gripper.opening_fit(np.deg2rad(angle)):.2f}%, derivative: {gripper.opening_derivative(np.deg2rad(angle)):.2f}, correction_factor: {gripper.get_correction_factor(np.deg2rad(angle)):.2f}"
        )

    print("\nTesting the  get_error_to_apply_force function")
    forces = [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30]
    for angle in test_values:
        for force in forces:
            error = gripper.get_error_to_apply_force(angle, force)
            print(
                f"Angle: {np.rad2deg(angle):.0f}°, Force: {force}N, Error: {np.rad2deg(error):.0f}°"
            )

    # Torque values from 0N to 40N
    torques = np.linspace(0, 40, 100)

    # Goal positions
    goal_positions = [0, np.deg2rad(50), np.deg2rad(120)]

    # Plotting
    plt.figure(figsize=(12, 8))

    for goal_position in goal_positions:
        errors = [
            gripper.get_error_to_apply_force(goal_position, torque)
            for torque in torques
        ]
        plt.plot(
            torques,
            np.rad2deg(errors),
            label=f"Goal Position = {np.rad2deg(goal_position):.2f}°",
        )

    plt.title("Error to Apply Force vs. Goal Torque")
    plt.xlabel("Goal Torque (N)")
    plt.ylabel("Error (degrees)")
    plt.legend()
    plt.grid(True)
    plt.show()
