from Player import Player
from Ball import Ball
from Window import Window
from Score import Score
from Settings import Action, Direction

from time import sleep

class Game:
    def __init__(self):
        self.left_player = Player()
        self.right_player = Player()
        self.ball = Ball()

        self.screen = Window()
        self.score = Score()
        # self.prediction_left = Instructor(Direction.LEFT)
        # self.prediction_right = Instructor(Direction.RIGHT)

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
            sleep(1)
            if self.run():
                self.screen.close_image()
        else:
            l, r = self.score.get_score()
            print("Game over\n  Final score is {}:{}\n  {} player wins".format(l, r, "Left" if l > r else "Right"))
            self.screen.close_image()

    def run(self):
        while True:
            # sleep(1/60)
            if not hasattr(self.ball, "nextnext_horizontal_collision"):
                self.ball.make_prediction()
  
            player_left_y = self.left_player.move_player_auto(
                self.ball.next_horizontal_collision
            )
            player_right_y = self.right_player.move_player_auto(
                self.ball.next_horizontal_collision
            )
            action = self.ball.move_ball(player_left_y, player_right_y)

            self.screen.reset_window()
            self.screen.draw_ball(*self.ball.get_position())
            self.screen.draw_left_player(player_left_y)
            self.screen.draw_right_player(player_right_y)
            self.screen.draw_score(*self.score.get_score())

            if not self.screen.show_image():
                return True
            if action == Action.PLAYER_LEFT_SCORED:
                self.score.player_left_scored()
                break
            elif action == Action.PLAYER_RIGHT_SCORED:
                self.score.player_right_scored()
                break
            
        self.setup()

if __name__ == "__main__":
    game = Game()
    game.setup()