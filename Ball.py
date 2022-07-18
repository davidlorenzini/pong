from math import pi, sin, cos, atan2
import random
from Settings import Settings, Collision, Action, Direction

class Ball(Settings):
    def __init__(self):
        super().__init__()
        self.angle = 0
        self.position_x = int((self.window_width - (self.ball_dimension - 1))/2)
        self.position_y = int((self.window_height - (self.ball_dimension - 1))/2)
        self.original_speed = self.ball_speed

    def reset_ball(self):
        self.position_x = int((self.window_width - (self.ball_dimension - 1))/2)
        self.position_y = int((self.window_height - (self.ball_dimension - 1))/2)
        self.angle = self._random_angle()
        self.ball_speed = self.original_speed
        
    def _random_angle(self):
        # Range: [-pi/4 | pi/4]
        # -45° bis 45°
        angle = random.random() * pi/2 - pi/4
        if random.random() * 2 > 1:
            # Range: [3/4*pi | 5/4*pi]
            # (90+45)° bis (180+45)°
            return angle + pi
        return angle

    def move_ball(self, player_left_y: int, player_right_y: int):
        self.position_x += cos(self.angle) * self.ball_speed
        # Y-direction has to be flipped, as (0,0) is top-left, not bottom-left
        self.position_y -= sin(self.angle) * self.ball_speed
        # print("x: {}, y: {}".format(self.position_x, self.position_y))
        res = self._check_collision()
        if res:
            case, direction = res
            if case == Collision.VERTICAL:
                # Express angle/vector as complex number a + ib
                # Mirror along x-axis (conjugate, a - ib)

                prev_angle = self.angle
                a = cos(self.angle)
                b = -sin(self.angle)
                self.angle = atan2(b,a)
                self.ball_speed += self.ball_accelerator
                # print("Vertical collision, old angle: {}, new angle: {}".format(prev_angle/pi *180, self.angle/pi *180))
            
            elif case == Collision.HORIZONTAL:
                res = self._check_for_score(direction, player_left_y, player_right_y)
                if res == Action.PLAYER_LEFT_SCORED: return Action.PLAYER_LEFT_SCORED
                elif res == Action.PLAYER_RIGHT_SCORED: return Action.PLAYER_RIGHT_SCORED

                self.make_prediction() 

                # Mirror it along x-axis and accross the origin (-conjugate, -a + ib)
                prev_angle = self.angle
                a = -cos(self.angle)
                b = sin(self.angle)
                self.angle = atan2(b,a)
                self.ball_speed += self.ball_accelerator
                
                # print("Horizontal collision, old angle: {}, new angle: {}".format(prev_angle/pi *180, self.angle/pi *180))
        return Action.GAME_CONTINUES

    def make_prediction(self):
        position_x = self.position_x
        position_y = self.position_y
        angle = self.angle
        ball_speed = self.ball_speed
        while True:
            position_x += cos(angle) * self.ball_speed
            # Y-direction has to be flipped, as (0,0) is top-left, not bottom-left
            position_y -= sin(angle) * self.ball_speed
            res = self._check_collision(position_x, position_y)
            if res:
                case, _ = res
                if case == Collision.VERTICAL:
                    a = cos(angle)
                    b = -sin(angle)
                    angle = atan2(b,a)
                    ball_speed += self.ball_accelerator
                
                elif case == Collision.HORIZONTAL:
                    self.next_horizontal_collision = position_y
                    break

    def _check_for_score(self, direction: Direction, player_left_y: int, player_right_y: int):
        if direction == Direction.LEFT:
            if (
                self.position_y + int((self.ball_dimension - 1)/2) < player_left_y - int((self.bar_height - 1)/2) or
                self.position_y - int((self.ball_dimension - 1)/2) > player_left_y + int((self.bar_height - 1)/2)
            ): return Action.PLAYER_RIGHT_SCORED
        elif direction == Direction.RIGHT:
            if (
                self.position_y + int((self.ball_dimension - 1)/2) < player_right_y - int((self.bar_height - 1)/2) or
                self.position_y - int((self.ball_dimension - 1)/2) > player_right_y + int((self.bar_height - 1)/2)
            ): return Action.PLAYER_LEFT_SCORED
        return Action.GAME_CONTINUES

    def _check_collision(self, position_x = None, position_y = None):
        if position_x == None:
            position_x = self.position_x 
        if position_y == None:
            position_y = self.position_y 
        if position_y + int((self.ball_dimension - 1)/2) >= self.window_height:
            return Collision.VERTICAL, Direction.DOWN
        elif position_y - int((self.ball_dimension - 1)/2) <= 0:
            return Collision.VERTICAL, Direction.UP
        elif position_x + int((self.ball_dimension - 1)/2) >= self.window_width - self.bar_width:
            return Collision.HORIZONTAL, Direction.RIGHT
        elif position_x - int((self.ball_dimension - 1)/2) <= self.bar_width:
            return Collision.HORIZONTAL, Direction.LEFT
        return False

    def get_position(self):
        return (self.position_x, self.position_y)
