from reachy2_sdk import ReachySDK

print('Connecting to Reachy...')
reachy = ReachySDK(host='localhost')
print('Turn_off_smoothly...')
reachy.turn_off_smoothly()