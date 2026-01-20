# Player control message. At the moment, the only implemented actions
# are MOVE and KICK. Others may be added as needed.
# Combined actions by ORing the constants, e.g. MOVE | KICK
uint16 NONE = 0
uint16 MOVE = 1
uint16 KICK = 2
uint16 action
# For the case when action includes MOVE
handball_msgs/PlanarTwist twist
# For the case when action includes KICK
handball_msgs/BallVelocity vel
