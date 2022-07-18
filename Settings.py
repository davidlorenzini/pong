from enum import IntEnum

class Direction(IntEnum):
    NONE = 0
    UP = -1
    DOWN =  1 
    LEFT  = -2
    RIGHT = 2


class Collision(IntEnum):
    VERTICAL = -1
    HORIZONTAL = 1


class Action(IntEnum):
    GAME_CONTINUES = 0
    PLAYER_LEFT_SCORED = 1
    PLAYER_RIGHT_SCORED = 2


class Settings:
    def __init__(self):
        self.bar_height = 200
        self.bar_width = 20
        self.bar_speed = 20
        
        self.ball_dimension = self.bar_width
        self.ball_speed = 5
        self.ball_accelerator = 1

        self.window_height = 640
        self.window_width = 1024