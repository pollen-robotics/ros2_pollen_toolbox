import time

from reachy2_sdk import ReachySDK

reachy = ReachySDK(host="localhost")
reachy.turn_on()
while True:
    reachy.mobile_base.goto(x=0, y=0.5, theta=0)
    time.sleep(2)
    reachy.mobile_base.goto(x=0.5, y=0.5, theta=0)
    time.sleep(2)
    reachy.mobile_base.goto(x=0.5, y=0, theta=0)
    time.sleep(2)
    reachy.mobile_base.goto(x=0, y=0, theta=0)
    time.sleep(2)
