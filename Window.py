import numpy as np
import cv2

from Settings import Settings

class Window(Settings):
    def __init__(self):
        super().__init__()
        self.image = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8) 

    def show_image(self):
        cv2.imshow("Pong", self.image)
        key = cv2.waitKey(3)
        if key != -1: 
            print(key, chr(key))
            if chr(key) == "q":
                return False
        return True
    
    def close_image(self, wait = 0):
        cv2.waitKey(wait)
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
