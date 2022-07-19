from enum import IntEnum
from turtle import window_height

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
        self.window_height = 600
        self.window_width = 1024

        self.bar_height = 200
        self.bar_width = 30
        self.bar_speed = int(self.window_height / 15)
        
        self.ball_dimension = self.bar_width
        self.ball_speed = 10
        self.ball_accelerator = 1
        self.max_ball_speed = int(self.window_width / 20)