from Player import Player
from Ball import Ball
from Window import Window
from Score import Score
from Navigator import Navigator
from Settings import Action
import argparse


from time import sleep

import rospy

class Game:
    def __init__(self, left_auto, right_auto):
        self.left_player = Player(None if left_auto else Navigator("right"))
        self.left_player_auto = left_auto
        self.right_player = Player(None if right_auto else Navigator("head"))
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
            self.run()
            #self.screen.close_image()
        else:
            l, r = self.score.get_score()
            print("Game over\n  Final score is {}:{}\n  {} player wins".format(l, r, "Left" if l > r else "Right"))
            #self.screen.close_image()

    def run(self):
        while True:
            if not hasattr(self.ball, "next_horizontal_collision"):
                self.ball.make_prediction()
            
            if self.left_player_auto:
                player_left_y = self.left_player.move_player_auto(self.ball.next_horizontal_collision)
            else:
                player_left_y = self.left_player.move_player()
            
            if self.right_player_auto:
                player_right_y = self.right_player.move_player_auto(self.ball.next_horizontal_collision)
            else:
                player_right_y = self.right_player.move_player()

            action = self.ball.move_ball(player_left_y, player_right_y)
            
            self.screen.reset_window()
            self.screen.draw_ball(*self.ball.get_position())
            self.screen.draw_left_player(player_left_y)
            self.screen.draw_right_player(player_right_y)
            self.screen.draw_score(*self.score.get_score())

            if not self.screen.display_image():
                return True
            if action == Action.PLAYER_LEFT_SCORED:
                self.score.player_left_scored()
                break
            elif action == Action.PLAYER_RIGHT_SCORED:
                self.score.player_right_scored()
                break
            
        self.setup()




if __name__ == "__main__":
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=Game.__doc__)
    parser.add_argument("-l", "--left_auto", dest="left_auto", action="store_true")
    parser.add_argument("-r", "--right_auto", dest="right_auto", action="store_true")
    
    rospy.init_node("Pong", anonymous=True)
    args = parser.parse_args(rospy.myargv()[1:])
    game = Game(args.left_auto, args.right_auto)
    game.setup()