from reachy2_sdk import ReachySDK

print('Connecting to Reachy...')
reachy = ReachySDK(host='localhost')
print('Turn_on')
reachy.turn_on()
