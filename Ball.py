from math import pi, sin, cos, atan2
import random
from Settings import Settings, Collision, Action, Direction

class Ball(Settings):
    def __init__(self):
        Settings.__init__(self)
        self.angle = 0
        self.position_x = int((self.window_width - (self.ball_dimension - 1))/2)
        self.position_y = int((self.window_height - (self.ball_dimension - 1))/2)

        self.original_speed = self.ball_speed

        self.left_prediction = False
        self.right_prediction = False

    def reset_ball(self):
        self.position_x = int((self.window_width - (self.ball_dimension - 1))/2)
        self.position_y = int((self.window_height - (self.ball_dimension - 1))/2)
        self.angle = self._random_angle()
        self.ball_speed = self.original_speed
        
    def _random_angle(self):
        # Range [pi/12 | pi/4] und [-pi/4 | -pi/12]
        angle = random.random() * pi/6 + pi/12
        angle = angle * ((-1) ** random.randint(0,1))
        if random.random() >= 0.5:
            return angle + pi
        return angle

    def move_ball(self, player_left_y, player_right_y):
        self.position_x += cos(self.angle) * self.ball_speed
        # Y-direction has to be flipped, as (0,0) is top-left, not bottom-left
        self.position_y -= sin(self.angle) * self.ball_speed
        
        res = self._check_collision()
        if res:
            case, direction = res
            if case == Collision.VERTICAL:
                # Express angle/vector as complex number a + ib
                # Mirror along x-axis (conjugate, a - ib)
                a = cos(self.angle)
                b = -sin(self.angle)
                self.angle = atan2(b,a)
                
                #Increase speed of ball
                if self.ball_speed < self.max_ball_speed: self.ball_speed += self.ball_accelerator
                
                # Make sure ball doesnt get out of bounds
                if self.position_y > self.window_height / 2:
                    self.position_y = self.window_height - int((self.ball_dimension - 1) / 2)
                else:
                    self.position_y = int((self.ball_dimension - 1) / 2)
            
            elif case == Collision.HORIZONTAL:
                res = self._check_for_score(direction, player_left_y, player_right_y)
                if res == Action.PLAYER_LEFT_SCORED: return Action.PLAYER_LEFT_SCORED
                elif res == Action.PLAYER_RIGHT_SCORED: return Action.PLAYER_RIGHT_SCORED

                # Reset predictions
                self.left_prediction = False
                self.right_prediction = False                

                # Mirror it along x-axis and accross the origin (-conjugate, -a + ib)
                prev_angle = self.angle
                a = -cos(self.angle)
                b = sin(self.angle)
                self.angle = atan2(b,a)

                if self.ball_speed < self.max_ball_speed: self.ball_speed += self.ball_accelerator

                if self.position_x > self.window_width / 2:
                    self.position_x = self.window_width - self.bar_width - int((self.ball_dimension - 1) / 2)
                else:
                    self.position_x = self.bar_width + int((self.ball_dimension - 1) / 2)
                    
        return Action.GAME_CONTINUES

    def make_prediction(self, player):
        position_x = self.position_x
        position_y = self.position_y
        angle = self.angle
        ball_speed = self.ball_speed

        # Return cached prediction, save the while loop
        if player == "left" and type(self.left_prediction) is int:
            return self.left_prediction
        if player == "right" and type(self.right_prediction) is int:
            return self.right_prediction

        while True:
            # Basically self.move_ball
            position_x += cos(angle) * self.ball_speed
            position_y -= sin(angle) * self.ball_speed
        
            res = self._check_collision(position_x, position_y)
            if res != False:
                case = res[0]
                if case == Collision.VERTICAL:
                    a = cos(angle)
                    b = -sin(angle)
                    angle = atan2(b,a)

                    if ball_speed < self.max_ball_speed: ball_speed += self.ball_accelerator
                    if position_y > self.window_height / 2:
                        position_y = self.window_height - int((self.ball_dimension - 1) / 2)
                    else:
                        position_y = int((self.ball_dimension - 1) / 2)
                
                elif case == Collision.HORIZONTAL:
                    # Save prediction
                    if position_x < self.window_width/2 and player == "left":
                        self.left_prediction = position_y
                        return position_y
                    elif position_x > self.window_width/2 and player == "right":
                        self.right_prediction = position_y
                        return position_y
                    else:
                        a = -cos(angle)
                        b = sin(angle)
                        angle = atan2(b,a)

                        if ball_speed < self.max_ball_speed: ball_speed += self.ball_accelerator

                        if position_x > self.window_width / 2:
                            position_x = self.window_width - self.bar_width - int((self.ball_dimension - 1) / 2)
                        else:
                            position_x = self.bar_width + int((self.ball_dimension - 1) / 2)

    def _check_for_score(self, direction, player_left_y, player_right_y):
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