from reachy2_sdk import ReachySDK
import logging
import time

if __name__ == "__main__":
    print("Reachy 2 test: torques")

    logging.basicConfig(level=logging.INFO)
    reachy = ReachySDK(host="localhost")

    if not reachy.is_connected:
        exit("Reachy is not connected.")

    print("Turning on Reachy")
    reachy.turn_on()
    time.sleep(0.2)
    assert reachy.is_on, "Reachy is not turned on."
    time.sleep(0.2)

    try:
        iter = 0

        while iter < 20:
            iter += 1
            print("Iteration: ", iter)
            reachy.r_arm.goto_posture()
            reachy.l_arm.goto_posture()
            reachy.r_arm.gripper.open()
            reachy.l_arm.gripper.open()
            head_goto = reachy.head.look_at(0.5, 10, 0)
            reachy.head.l_antenna.goto_position(-10)
            reachy.head.r_antenna.goto_position(10)

            while not reachy.head.is_goto_finished(head_goto):
                time.sleep(0.1)
            time.sleep(0.5)

            reachy.r_arm.goto_posture("elbow_90")
            reachy.l_arm.goto_posture("elbow_90")
            reachy.r_arm.gripper.close()
            reachy.l_arm.gripper.close()
            head_goto = reachy.head.look_at(0.5, -20, 0)
            reachy.head.l_antenna.goto_position(30)
            reachy.head.r_antenna.goto_position(-30)

            while not reachy.head.is_goto_finished(head_goto):
                time.sleep(0.1)
            time.sleep(0.5)

            reachy.turn_off_smoothly()
            time.sleep(0.5)
            assert reachy.is_off, "Reachy is not turned off."

    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("Set to Zero pose ...")
        goto_ids = reachy.goto_posture("default", wait=True)
        reachy.r_arm.gripper.open()
        reachy.l_arm.gripper.open()
        reachy.cancel_all_gotos()

        print("Turning off Reachy")
        reachy.turn_off_smoothly()

        time.sleep(0.2)

        exit("Exiting torques test")
