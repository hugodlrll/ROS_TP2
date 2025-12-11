# FOUL_PLAY may be implemented in the future, and would 
# typically require to set the ball at a given position
# for the benefit of a given team. But for now, those are unused.
uint8 START_PLAY = 1
uint8 SUSPEND_PLAY = 2
uint8 END_OF_GAME = 3
uint8 FOUL_PLAY = 10
uint8 why
handball_msgs/BallPosition position
string team
