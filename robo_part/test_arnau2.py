# Import necessary classes from LeRobot
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

import readchar
import time

leader_config = SO101LeaderConfig(
    port="/dev/ttyACM1",
    id="my_controller",
)


device = SO101Leader(leader_config)
device.connect()

while True:
    action = device.get_action()
    print(action)
    time.sleep(5.0) 



# action = {
#     'shoulder_pan.pos': 70.29491424560547,
#     'shoulder_lift.pos': -40.86470031738281,
#     'elbow_flex.pos': 92.48854064941406,
#     'wrist_flex.pos': 1.1112022399902344,
#     'wrist_roll.pos': 0,
#     'gripper.pos': 2.0
# }

# action = {'shoulder_pan.pos': 81.15035247802734, 
# 'shoulder_lift.pos': -100.24190521240234, 
# 'elbow_flex.pos': 170.06210327148438, 
# 'wrist_flex.pos': 5.187788963317871, 
# 'wrist_roll.pos': 0.0, 
# 'gripper.pos': 2.0}

# follower_config = SO101FollowerConfig(
#     port="/dev/ttyACM1",
#     id="my_robot_arm",
# )   

# device = SO101Follower(follower_config)
# device.connect()


# # action = {'shoulder_pan.pos': -100.0, 'shoulder_lift.pos': 68.9246401354784, 'elbow_flex.pos': -36.098450319051956, 'wrist_flex.pos': -1.6115351993214517, 'wrist_roll.pos': -51.61290322580645, 'gripper.pos': 1.7484868863483525}


# while True:
    

#     device.send_action(action)

# # shoulder_pan.pos = (-100, 100)
# # shoulder_lift.pos = (-100.0, 100.0)
# # elbow_flex.pos = (-100.0, 100.0)
# # wrist_flex.pos = (-100.0, 100.0)
# # wrist_roll.pos = (-100.0, 100.0)
# # gripper.pos = (1.0, 100.0)
