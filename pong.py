import numpy as np
from PIL import Image
from os.path import dirname,abspath
from math import floor
import cv2
import time
from math import tan, atan

class Pong:
    def __init__(self):

        ratio = 16/9
        self.page_h = 1000
        self.page_w = floor(ratio * self.page_h)
        print(str(self.page_w) + "x" + str(self.page_h))
        self.board_h = 200
        self.board_w = floor(self.board_h/5)

        self.ball_dim = self.board_w

        self.im = np.zeros((self.page_h, self.page_w, 3), dtype=np.uint8)

        self.positions = {"p1": (), "p2": (), "b": ()}
        self.velocity = (0,0)
        self.score = (0,0)
        self.playing = False

    def draw(self, x, y, w, h, color=255):
        if self.page_h < y + h:
            if y > self.page_h:
                raise Exception("y out of range: {} > {}".format(y, self.page_h))
                
            h = self.page_h - y

        if self.page_w < x + w:

            if x > self.page_w:
                raise Exception("x out of range: {} > {}".format(x, self.page_w))
                
            w = self.page_w - x
        self.im[y:y+h, x:x+w] = [color,color,color]

    def draw_board(self, x, y, player):
        self.positions[player] = (x,y)
        self.draw(floor(x), floor(y), self.board_w, self.board_h)

    def draw_ball(self, prevx, prevy,):
        x,y = self.positions["b"]
        # self.draw(floor(prevx), floor(prevy), self.ball_dim, self.ball_dim, color=0)
        self.draw(floor(x), floor(y), self.ball_dim, self.ball_dim)

    def setup(self):
        # self.draw_board(0, (p.page_h - p.board_h)/2, "p1")
        # self.draw_board(self.page_w - self.board_w, (p.page_h - p.board_h)/2, "p2")
        self.positions["b"] = ((self.page_w - self.ball_dim)/2, (self.page_h - self.ball_dim)/2)
        self.draw_ball(0,0)
        self.velocity = (self.page_w/ 20, self.page_h/20)
        cv2.namedWindow("pong", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("pong", self.im)
        cv2.waitKey(3)

    def run(self):
        count = 0
        while count < 20:
            x1, y1, vx1, vy1 = 0, 0, 0, 0
            x,y = self.positions["b"]
            prevx, prevy = x, y
            vx, vy = self.velocity
            print("x: {}, y: {}, vx: {}, vy: {}".format(x,y,vx,vy))
            x,y = floor(x + vx), floor(y + vy)
            print(x,y)
            # left out of bounds
            if x <= 0:
                # x1, y1, vx1, vy1 = x1, y1, vx1, vy1
                if x < 0:
                    x1, y1 = 0, floor(y + (tan(atan(vy/vx)) * (-x)))
                vx1, vy1 = -vx, vy
            
            # top out of bounds
            if y <= 0:
                # x1, y1, vx1, vy1 = x1, y1, vx1, vy1
                dx = 0
                if y < 0:
                    dx = (-y) / tan(atan(vy/vx))

                    #Top-left edge case
                    if floor(x + dx) >= 0:
                        x1, y1 = floor(x + dx), 0
                if floor(x + dx) > 0:
                    vx1, vy1 = vx, -vy
                elif floor(x + dx) == 0:
                    vx1, vy1 = -vx, -vy
            
            # right out of bounds
            if x + self.ball_dim >= self.page_w:
                dy = 0
                if x + self.ball_dim > self.page_w:
                    dy = (tan(atan(vy/vx)) * (x + self.ball_dim - self.page_w))
                    
                    #Top-right edge case
                    if floor(y + dy) >= 0:
                        x1, y1 = floor(y + dy), self.page_w - self.ball_dim
                if floor(y + dy) > 0:
                    vx1, vy1 = -vx, vy
                elif floor(y + dy) == 0:
                    vx1, vy1 = -vx, -vy
            
            # bottom out of bounds
            if y + self.ball_dim >= self.page_h:
                if y + self.ball_dim > self.page_h:
                    dx = (y + self.ball_dim - self.page_h) / tan(atan(vx/vy))
                    x1, y1 = floor(x + dx + self.ball_dim), self.page_h - self.ball_dim

                    #Top-left edge case
                    if floor(x + dx) >= 0:
                        x1 = floor(x + dx)
                        # y1 = 0 if 
                if floor(x + dx) > 0:
                    vx1, vy1 = vx, -vy
                elif floor(x + dx) == 0:
                    vx1, vy1 = -vx, -vy
                vx1, vy1 = vx, -vy
            
            if x1 == 0 and y1 == 0 and vx1 == 0 and vy1 == 0:
                x1, y1, vx1, vy1 = x, y, vx, vy
            
            print("x1: {}, y1: {}, vx1: {}, vy1: {}".format(x1,y1,vx1,vy1))
            
            self.positions["b"] = (x1, y1)
            self.velocity = (vx1, vy1)
            self.draw_ball(prevx, prevy)
            
            cv2.imshow("pong", self.im)
            cv2.waitKey(3)
            count += 1
            time.sleep(0.5)
        cv2.waitKey(0)
        cv2.destroyAllWindows()




    def save(self):
        path = dirname(abspath(__file__)) + "\\\\pong.png"
        img = Image.fromarray(self.im, "RGB").save(path)


p = Pong()
p.setup()
p.run()

# x: 1685, y: 132, vx: 88.85, vy: 33.333333333333336
#x1: 1737, y1: 69, vx1: -88.85, vy1: 33.333333333333336