from reachy2_sdk import ReachySDK
import logging
import time

if __name__ == "__main__":
    print("Reachy 2 test: torques")

    test_result = "SUCCEEDED"

    logging.basicConfig(level=logging.INFO)
    reachy = ReachySDK(host="localhost")

    if not reachy.is_connected:
        exit("Reachy is not connected.")

    try:
        print("Turning on Reachy")
        reachy.turn_on()
        time.sleep(0.2)
        assert reachy.is_on()
        time.sleep(0.2)

        iter = 0

        while iter < 20:
            iter += 1
            print("Iteration: ", iter)
            reachy.r_arm.goto_posture()
            reachy.l_arm.goto_posture()
            reachy.r_arm.gripper.open()
            reachy.l_arm.gripper.open()
            head_goto = reachy.head.look_at(0.5, 0, 10)
            reachy.head.l_antenna.goto(-20)
            reachy.head.r_antenna.goto(20)

            while not reachy.is_goto_finished(head_goto):
                time.sleep(0.1)
            time.sleep(0.5)

            reachy.r_arm.goto_posture("elbow_90")
            reachy.l_arm.goto_posture("elbow_90")
            reachy.r_arm.gripper.close()
            reachy.l_arm.gripper.close()
            head_goto = reachy.head.look_at(0.5, 0, -20)
            reachy.head.l_antenna.goto(50)
            reachy.head.r_antenna.goto(-50)

            while not reachy.is_goto_finished(head_goto):
                time.sleep(0.1)
            time.sleep(0.5)

            reachy.turn_off_smoothly()
            time.sleep(0.5)
            assert reachy.is_off()

    except Exception as e:
        print(f"An error occurred: {e}")
        test_result = "FAILED"
        import traceback

        traceback.print_exc()
    finally:
        print("Set to Zero pose ...")
        goto_ids = reachy.goto_posture("default", wait=True)
        reachy.r_arm.gripper.open()
        reachy.l_arm.gripper.open()
        reachy.cancel_all_goto()

        print("Turning off Reachy")
        reachy.turn_off_smoothly()

        time.sleep(0.2)

        exit("Exiting torques test with result: " + test_result)
