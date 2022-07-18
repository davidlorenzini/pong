env = "Sawyer"
from Player import Player
from Ball import Ball
from Window import Window
from Score import Score
from Controller import Navigator, Keyboard
from Settings import Action
import argparse
from time import sleep
from sys import argv
try:
    import rospy
except ModuleNotFoundError:
    env = "PC"

class Game:
    def __init__(self, left_auto, right_auto, use_keyboard):
        # Set controls fro left player
        if use_keyboard: left_controller = Keyboard("w", "s")
        else: left_controller = Navigator("right")
        self.left_player = Player(left_controller)
        self.left_player_auto = left_auto

        # Set controls right player
        if use_keyboard: right_controller = Keyboard("o", "k")
        else: right_controller = Navigator("head")
        self.right_player = Player(right_controller)
        self.right_player_auto = right_auto

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
        
        self.screen.display_image()
        
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

            if self.left_player_auto:
                player_left_y = self.left_player.move_player_auto(self.ball.make_prediction("left"))
            else:
                player_left_y = self.left_player.move_player()
            
            if self.right_player_auto:
                player_right_y = self.right_player.move_player_auto(self.ball.make_prediction("right"))
            else:
                player_right_y = self.right_player.move_player()

            action = self.ball.move_ball(player_left_y, player_right_y)
            
            self.screen.reset_window()
            self.screen.draw_ball(*self.ball.get_position())
            self.screen.draw_left_player(player_left_y)
            self.screen.draw_right_player(player_right_y)
            self.screen.draw_score(*self.score.get_score())

            res = self.screen.display_image()
            if type(res) is bool:
                # Close the game
                if not res: return True
            elif env == "PC":
                # Send key to controller
                if self.left_player.controller.key_pressed(res) == True: return True
                if self.right_player.controller.key_pressed(res) == True: return True


            if action == Action.PLAYER_LEFT_SCORED:
                self.score.player_left_scored()
                break
            elif action == Action.PLAYER_RIGHT_SCORED:
                self.score.player_right_scored()
                break
            
        self.setup()


if __name__ == "__main__":
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument("-l", "--left_auto", dest="left_auto", action="store_true")
    parser.add_argument("-r", "--right_auto", dest="right_auto", action="store_true")
    parser.add_argument("-k", "--use_keyboard", dest="use_keyboard", action="store_true")
    
    if env == "Sawyer": 
        rospy.init_node("Pong", anonymous=True)
        args = parser.parse_args(rospy.myargv()[1:])
    else:
        args = parser.parse_args(argv[1:])
    game = Game(args.left_auto, args.right_auto, args.use_keyboard)
    game.setup()