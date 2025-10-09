from reachy2_sdk import ReachySDK
import time
import numpy as np
import pytest

N = 20


@pytest.fixture(scope="package")
def reachy_sdk() -> ReachySDK:
    reachy = ReachySDK(host="localhost")
    assert reachy.is_connected()

    try:
        yield reachy

    finally:
        reachy.goto_posture("default", wait=True)
        reachy.cancel_all_goto()

        reachy.turn_off_smoothly()

    reachy.disconnect()


@pytest.mark.parametrize("iter_idx", range(N))
def test_torque_and_pos(reachy_sdk: ReachySDK, iter_idx) -> None:
    reachy_sdk.turn_on()
    time.sleep(0.2)
    assert reachy_sdk.is_on()

    reachy_sdk.r_arm.goto_posture()
    reachy_sdk.l_arm.goto_posture()
    reachy_sdk.r_arm.gripper.open()
    reachy_sdk.l_arm.gripper.open()
    head_goto = reachy_sdk.head.goto([0, 0, -15])
    reachy_sdk.head.l_antenna.goto(-20, duration=0.5)
    reachy_sdk.head.r_antenna.goto(20, duration=0.5)
    reachy_sdk.head.l_antenna.goto(20, duration=0.5)
    reachy_sdk.head.r_antenna.goto(-20, duration=0.5)
    reachy_sdk.head.l_antenna.goto(-20, duration=0.5)
    reachy_sdk.head.r_antenna.goto(20, duration=0.5)

    while not reachy_sdk.is_goto_finished(head_goto):
        time.sleep(0.1)
    assert np.isclose(reachy_sdk.head.l_antenna.present_position, -20, atol=1)
    assert np.isclose(reachy_sdk.head.r_antenna.present_position, 20, atol=1)
    assert np.allclose(
        reachy_sdk.r_arm.get_current_positions(),
        reachy_sdk.r_arm.get_default_posture_joints(),
        atol=0.5,
    )
    assert np.allclose(
        reachy_sdk.l_arm.get_current_positions(),
        reachy_sdk.l_arm.get_default_posture_joints(),
        atol=0.5,
    )
    assert np.allclose(reachy_sdk.head.get_current_positions(), [0, 0, -15], atol=0.1)
    assert np.isclose(reachy_sdk.r_arm.gripper.opening, 100, atol=0.2)
    assert np.isclose(reachy_sdk.r_arm.gripper.opening, 100, atol=0.2)
    time.sleep(0.5)

    reachy_sdk.r_arm.goto_posture("elbow_90")
    reachy_sdk.l_arm.goto_posture("elbow_90")
    reachy_sdk.r_arm.gripper.close()
    reachy_sdk.l_arm.gripper.close()
    head_goto = reachy_sdk.head.goto([0, 30, 0])
    reachy_sdk.head.l_antenna.goto(50, duration=0.5)
    reachy_sdk.head.r_antenna.goto(-50, duration=0.5)
    reachy_sdk.head.l_antenna.goto(0, duration=0.5)
    reachy_sdk.head.r_antenna.goto(0, duration=0.5)
    reachy_sdk.head.l_antenna.goto(50, duration=0.5)
    reachy_sdk.head.r_antenna.goto(-50, duration=0.5)

    while not reachy_sdk.is_goto_finished(head_goto):
        time.sleep(0.1)
    assert np.isclose(reachy_sdk.head.l_antenna.present_position, 50, atol=1)
    assert np.isclose(reachy_sdk.head.r_antenna.present_position, -50, atol=1)
    assert np.allclose(
        reachy_sdk.r_arm.get_current_positions(),
        reachy_sdk.r_arm.get_default_posture_joints("elbow_90"),
        atol=0.5,
    )
    assert np.allclose(
        reachy_sdk.l_arm.get_current_positions(),
        reachy_sdk.l_arm.get_default_posture_joints("elbow_90"),
        atol=0.5,
    )
    assert np.isclose(reachy_sdk.r_arm.gripper.opening, 0, atol=0.2)
    assert np.isclose(reachy_sdk.r_arm.gripper.opening, 0, atol=0.2)
    assert np.allclose(reachy_sdk.head.get_current_positions(), [0, 30, 0], atol=0.1)
    time.sleep(0.5)

    reachy_sdk.turn_off_smoothly()
    time.sleep(0.2)
    assert reachy_sdk.is_off()
