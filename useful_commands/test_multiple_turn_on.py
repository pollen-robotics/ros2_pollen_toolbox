from reachy2_sdk import ReachySDK
import time
import numpy as np
import pytest

N = 20  # Number of iterations of the scenario

TOL_ANTENNA = 3  # degrees
TOL_GRIPPER = 1  # percentage
TOL_NECK = 0.1  # degrees
TOL_ARM = 0.5  # degrees
TOL_DURATION = 0.1  # seconds


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


def _run_scenario_once(reachy_sdk: ReachySDK):
    """Run the scenario once and return the results."""
    results = {}

    # ========= Turn on =========
    reachy_sdk.turn_on()
    time.sleep(0.2)
    results["is_on_start"] = reachy_sdk.is_on()

    # ========= Phase 1 =========
    r_arm_goto_p1 = reachy_sdk.r_arm.goto_posture()
    l_arm_goto_p1 = reachy_sdk.l_arm.goto_posture()
    r_gripper_goto_p1 = reachy_sdk.r_arm.gripper.goto(100, percentage=True)
    l_gripper_goto_p1 = reachy_sdk.l_arm.gripper.goto(100, percentage=True)

    reachy_sdk.head.l_antenna.goto(0, duration=0.3)
    reachy_sdk.head.r_antenna.goto(0, duration=0.3)
    l_antenna_goto_p1 = reachy_sdk.head.l_antenna.goto(
        -20, duration=1.5, interpolation_mode="linear"
    )
    r_antenna_goto_p1 = reachy_sdk.head.r_antenna.goto(
        20, duration=1.5, interpolation_mode="linear"
    )

    tic = time.time()
    reachy_sdk.head.goto([0, 0, -15], duration=2.0, wait=True)
    results["head_goto_duration_p1"] = time.time() - tic

    # Phase 1 goto results
    results["r_arm_goto_finished_p1"] = reachy_sdk.is_goto_finished(r_arm_goto_p1)
    results["l_arm_goto_finished_p1"] = reachy_sdk.is_goto_finished(l_arm_goto_p1)
    results["r_gripper_goto_finished_p1"] = reachy_sdk.is_goto_finished(
        r_gripper_goto_p1
    )
    results["l_gripper_goto_finished_p1"] = reachy_sdk.is_goto_finished(
        l_gripper_goto_p1
    )
    results["l_antenna_goto_finished_p1"] = reachy_sdk.is_goto_finished(
        l_antenna_goto_p1
    )
    results["r_antenna_goto_finished_p1"] = reachy_sdk.is_goto_finished(
        r_antenna_goto_p1
    )

    time.sleep(0.2)

    # Phase 1 position results
    results["ant_l_p1"] = reachy_sdk.head.l_antenna.present_position
    results["ant_r_p1"] = reachy_sdk.head.r_antenna.present_position
    results["r_arm_pos_p1"] = reachy_sdk.r_arm.get_current_positions()
    results["l_arm_pos_p1"] = reachy_sdk.l_arm.get_current_positions()
    results["r_arm_posture_def"] = reachy_sdk.r_arm.get_default_posture_joints()
    results["l_arm_posture_def"] = reachy_sdk.l_arm.get_default_posture_joints()
    results["head_pos_p1"] = reachy_sdk.head.get_current_positions()
    results["grip_r_p1"] = reachy_sdk.r_arm.gripper.opening
    results["grip_l_p1"] = reachy_sdk.l_arm.gripper.opening

    time.sleep(0.5)

    # ========= Phase 2 =========
    head_goto_p2 = reachy_sdk.head.goto([0, 30, 0])
    l_arm_goto_p2 = reachy_sdk.l_arm.goto_posture("elbow_90")
    r_gripper_goto_p2 = reachy_sdk.r_arm.gripper.goto(0, percentage=True)
    l_gripper_goto_p2 = reachy_sdk.l_arm.gripper.goto(0, percentage=True)

    reachy_sdk.head.l_antenna.goto(50, duration=0.5)
    reachy_sdk.head.r_antenna.goto(-50, duration=0.5)
    reachy_sdk.head.l_antenna.goto(0, duration=0.5)
    reachy_sdk.head.r_antenna.goto(0, duration=0.5)
    l_antenna_goto_p2 = reachy_sdk.head.l_antenna.goto(50, duration=0.5)
    r_antenna_goto_p2 = reachy_sdk.head.r_antenna.goto(-50, duration=0.5)

    tic = time.time()
    reachy_sdk.r_arm.goto_posture("elbow_90", duration=2.0, wait=True)
    results["r_arm_goto_duration_p2"] = time.time() - tic

    # Phase 2 goto results
    results["head_goto_finished_p2"] = reachy_sdk.is_goto_finished(head_goto_p2)
    results["l_arm_goto_finished_p2"] = reachy_sdk.is_goto_finished(l_arm_goto_p2)
    results["r_gripper_goto_finished_p2"] = reachy_sdk.is_goto_finished(
        r_gripper_goto_p2
    )
    results["l_gripper_goto_finished_p2"] = reachy_sdk.is_goto_finished(
        l_gripper_goto_p2
    )
    results["l_antenna_goto_finished_p2"] = reachy_sdk.is_goto_finished(
        l_antenna_goto_p2
    )
    results["r_antenna_goto_finished_p2"] = reachy_sdk.is_goto_finished(
        r_antenna_goto_p2
    )

    time.sleep(0.2)

    # Phase 2 position results
    results["ant_l_p2"] = reachy_sdk.head.l_antenna.present_position
    results["ant_r_p2"] = reachy_sdk.head.r_antenna.present_position
    results["r_arm_pos_p2"] = reachy_sdk.r_arm.get_current_positions()
    results["l_arm_pos_p2"] = reachy_sdk.l_arm.get_current_positions()
    results["r_arm_posture_elbow90"] = reachy_sdk.r_arm.get_default_posture_joints(
        "elbow_90"
    )
    results["l_arm_posture_elbow90"] = reachy_sdk.l_arm.get_default_posture_joints(
        "elbow_90"
    )
    results["grip_r_p2"] = reachy_sdk.r_arm.gripper.opening
    results["grip_l_p2"] = reachy_sdk.l_arm.gripper.opening
    results["head_pos_p2"] = reachy_sdk.head.get_current_positions()

    time.sleep(0.5)

    # ========= Turn off =========
    reachy_sdk.turn_off_smoothly()
    time.sleep(0.2)
    results["is_off_end"] = reachy_sdk.is_off()

    return results


@pytest.fixture(scope="module", params=range(N))
def scenario(request, reachy_sdk: ReachySDK):
    """
    Run the scenario **once** per iteration for the whole module,
        and share the measurements between tests. Thus, each test makes a single assertion,
        and a failure does not interrupt the others.
    """
    return _run_scenario_once(reachy_sdk)


# ---------- All tests are conducted N times ----------

# Torques on/off tests


def test_turn_on(scenario):
    assert scenario["is_on_start"]


def test_turn_off(scenario):
    assert scenario["is_off_end"]


# ========= Phase 1 =========

# Goto duration phase 1


def test_arm_right_goto_finished_phase1(scenario):
    assert scenario["r_arm_goto_finished_p1"]


def test_arm_left_goto_finished_phase1(scenario):
    assert scenario["l_arm_goto_finished_p1"]


def test_gripper_right_goto_finished_phase1(scenario):
    assert scenario["r_gripper_goto_finished_p1"]


def test_gripper_left_goto_finished_phase1(scenario):
    assert scenario["l_gripper_goto_finished_p1"]


def test_antenna_left_goto_finished_phase1(scenario):
    assert scenario["l_antenna_goto_finished_p1"]


def test_antenna_right_goto_finished_phase1(scenario):
    assert scenario["r_antenna_goto_finished_p1"]


def test_head_goto_duration_phase1(scenario):
    assert np.isclose(scenario["head_goto_duration_p1"], 2.0, atol=TOL_DURATION)


# Goto positions phase 1


def test_antennas_phase1_left(scenario):
    assert np.isclose(scenario["ant_l_p1"], -20, atol=TOL_ANTENNA)


def test_antennas_phase1_right(scenario):
    assert np.isclose(scenario["ant_r_p1"], 20, atol=TOL_ANTENNA)


def test_arm_right_default_posture(scenario):
    assert np.allclose(
        scenario["r_arm_pos_p1"], scenario["r_arm_posture_def"], atol=TOL_ARM
    )


def test_arm_left_default_posture(scenario):
    assert np.allclose(
        scenario["l_arm_pos_p1"], scenario["l_arm_posture_def"], atol=TOL_ARM
    )


def test_head_phase1(scenario):
    assert np.allclose(scenario["head_pos_p1"], [0, 0, -15], atol=TOL_NECK)


def test_gripper_phase1_right_open(scenario):
    assert np.isclose(scenario["grip_r_p1"], 100, atol=TOL_GRIPPER)


def test_gripper_phase1_left_open(scenario):
    assert np.isclose(scenario["grip_l_p1"], 100, atol=TOL_GRIPPER)


# ========= Phase 2 =========

# Goto duration phase 2


def test_head_goto_finished_phase2(scenario):
    assert scenario["head_goto_finished_p2"]


def test_arm_left_goto_finished_phase2(scenario):
    assert scenario["l_arm_goto_finished_p2"]


def test_gripper_right_goto_finished_phase2(scenario):
    assert scenario["r_gripper_goto_finished_p2"]


def test_gripper_left_goto_finished_phase2(scenario):
    assert scenario["l_gripper_goto_finished_p2"]


def test_antenna_left_goto_finished_phase2(scenario):
    assert scenario["l_antenna_goto_finished_p2"]


def test_antenna_right_goto_finished_phase2(scenario):
    assert scenario["r_antenna_goto_finished_p2"]


def test_r_arm_goto_duration_phase2(scenario):
    assert np.isclose(scenario["r_arm_goto_duration_p2"], 2.0, atol=TOL_DURATION)


# Goto position phase 2


def test_antennas_phase2_left(scenario):
    assert np.isclose(scenario["ant_l_p2"], 50, atol=TOL_ANTENNA)


def test_antennas_phase2_right(scenario):
    assert np.isclose(scenario["ant_r_p2"], -50, atol=TOL_ANTENNA)


def test_arm_right_elbow90_posture(scenario):
    assert np.allclose(
        scenario["r_arm_pos_p2"], scenario["r_arm_posture_elbow90"], atol=TOL_ARM
    )


def test_arm_left_elbow90_posture(scenario):
    assert np.allclose(
        scenario["l_arm_pos_p2"], scenario["l_arm_posture_elbow90"], atol=TOL_ARM
    )


def test_gripper_phase2_right_closed(scenario):
    assert np.isclose(scenario["grip_r_p2"], 0, atol=TOL_GRIPPER)


def test_gripper_phase2_left_closed(scenario):
    assert np.isclose(scenario["grip_l_p2"], 0, atol=TOL_GRIPPER)


def test_head_phase2(scenario):
    assert np.allclose(scenario["head_pos_p2"], [0, 30, 0], atol=TOL_NECK)
