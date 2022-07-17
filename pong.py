from enum import Enum
import numpy as np
from math import cos, sin, pi, atan2
import cv2
import time
import random

class Direction(Enum):
    UP = -1
    DOWN =  1 
    LEFT  = -1
    RIGHT = 1


class Collision(Enum):
    VERTICAL = 0
    HORIZONTAL = 1


class Action(Enum):
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


class Player(Settings):
    def __init__(self):
        super().__init__()
        self._actual_position = int(self.bar_height/2)

    def move_player(self, direction: Direction) -> int:
        self._actual_position += direction * self.bar_speed
        self._check_position()
        return self._actual_position

    def _check_position(self):
        if int(self._actual_position + self.bar_height/2) > self.window_height:
            self._actual_position = int(self.window_height - self.bar_height/2)
        elif int(self._actual_position - self.bar_height/2) < 0:
            self._actual_position = int(self.bar_height/2)

    def get_position(self):
        return self._actual_position

    def reset_player(self):
        self._actual_position = int((self.window_height - self.bar_height)/2)


class Score:
    def __init__(self):
        self.player_left = 0
        self.player_right = 0

    def player_left_scored(self):
        self.player_left += 1
        print("Left player scored\n  Score is now {}:{}".format(*self.get_score()))
    
    def player_right_scored(self):
        self.player_right += 1
        print("Right player scored\n  Score is now {}:{}".format(*self.get_score()))

    def get_score(self):
        return (self.player_left, self.player_right)


class Window(Settings):
    def __init__(self):
        super().__init__()
        self.image = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8) 

    def show_image(self):
        cv2.imshow("Pong", self.image)
        key = cv2.waitKey(3)
        if key != -1: print(chr(key))
    
    def close_image(self):
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def draw_right_player(self, position_y: int):
        cv2.rectangle(
            self.image,
            (self.window_width - self.bar_width, int(position_y - self.bar_height/2)),
            (self.window_width - 1, int(position_y + self.bar_height/2)),
            (255, 255, 255),
            -1
        )

    def draw_left_player(self, position_y: int):
        cv2.rectangle(
            self.image,
            (0, int(position_y - (self.bar_height - 1)/2)),
            (self.bar_width - 1, int(position_y + (self.bar_height - 1)/2)),
            (255, 255, 255),
            -1
        )

    def draw_ball(self, position_x: int, position_y: int):
        cv2.rectangle(
            self.image,
            (int(position_x - (self.ball_dimension - 1)/2), int(position_y - (self.ball_dimension - 1)/2)),
            (int(position_x + (self.ball_dimension - 1)/2), int(position_y + (self.ball_dimension - 1)/2)),
            (255, 255, 255),
            -1
        )

    def draw_score(self, score_left: int, score_right: int):
        font = cv2.FONT_HERSHEY_PLAIN
        text = str(score_left) + ":" + str(score_right)
        text_size = cv2.getTextSize(text, font, 2, 2)[0]
        text_x = int((self.window_width - text_size[0]) / 2)
        text_y = 70
        cv2.putText(
            self.image, 
            text, 
            (text_x,  text_y), 
            font, 
            2, 
            (255, 255, 255), 
            2
        )

    def reset_window(self):
        self.image = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8) 


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
                # Mirror along x-axis

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

                # Express angle/vector as complex number a + ib
                # Mirror it along x-axis (conjugate, a - ib) and accross the origin (-conjugate, -a + ib)
                prev_angle = self.angle
                a = -cos(self.angle)
                b = sin(self.angle)
                self.angle = atan2(b,a)
                self.ball_speed += self.ball_accelerator
                
                # print("Horizontal collision, old angle: {}, new angle: {}".format(prev_angle/pi *180, self.angle/pi *180))
        return Action.GAME_CONTINUES

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

    def _check_collision(self):
        if self.position_y + int((self.ball_dimension - 1)/2) >= self.window_height:
            return Collision.VERTICAL, Direction.DOWN
        elif self.position_y - int((self.ball_dimension - 1)/2) <= 0:
            return Collision.VERTICAL, Direction.UP
        elif self.position_x + int((self.ball_dimension - 1)/2) >= self.window_width - self.bar_width:
            return Collision.HORIZONTAL, Direction.RIGHT
        elif self.position_x - int((self.ball_dimension - 1)/2) <= self.bar_width:
            return Collision.HORIZONTAL, Direction.LEFT
        return False

    def get_position(self):
        return (self.position_x, self.position_y)


class Game:
    def __init__(self):
        self.left_player = Player()
        self.right_player = Player()
        self.ball = Ball()

        self.screen = Window()
        self.score = Score()


    def setup(self):
        # Set positions & image to default
        self.screen.reset_window()
        self.ball.reset_ball()
        self.left_player.reset_player()
        self.right_player.reset_player()

        # Draw to image
        self.screen.draw_ball(*self.ball.get_position())
        self.screen.draw_left_player(self.left_player.get_position())
        self.screen.draw_right_player(self.right_player.get_position())
        self.screen.draw_score(*self.score.get_score())
        
        self.screen.show_image()
        
        a,b=self.score.get_score()
        if a < 50 and b < 50:
            time.sleep(1)
            self.run()
        else:
            l, r = self.score.get_score()
            print("Game over\n  Final score is {}:{}\n  {} player wins".format(l, r, "Left" if l > r else "Right"))
            self.screen.close_image()

    def run(self):
        while True:
            # time.sleep(1/60)
            player_left_y = self.left_player.move_player((-1) ** random.randint(0,1))
            player_right_y = self.right_player.move_player((-1) ** random.randint(0,1))
            action = self.ball.move_ball(player_left_y, player_right_y)
            self.screen.reset_window()
            self.screen.draw_ball(*self.ball.get_position())
            self.screen.draw_left_player(player_left_y)
            self.screen.draw_right_player(player_right_y)
            self.screen.draw_score(*self.score.get_score())
            self.screen.show_image()
            if action == Action.PLAYER_LEFT_SCORED:
                self.score.player_left_scored()
                break
            elif action == Action.PLAYER_RIGHT_SCORED:
                self.score.player_right_scored()
                break
            
        self.setup()

    def close(self):
        self.screen.close_image()


if __name__ == "__main__":
    game = Game()
    game.setup()