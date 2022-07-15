import numpy as np
from PIL import Image
from os.path import dirname,abspath
from math import floor, tan, atan
import cv2
import time
from random import randint

class Pong:
    def __init__(self):
        ratio = 16/9
        self.page_h = 1000
        self.page_w = floor(ratio * self.page_h)
        print(str(self.page_w) + "x" + str(self.page_h))
        # 250x
        self.board_h2 = 124
        self.board_w2 = 14

        self.ball_dim = 14 # 25x25

        self.im = np.zeros((self.page_h, self.page_w, 3), dtype=np.uint8)

        self.positions = {"p1": (), "p2": (), "b": ()}
        self.velocity = (0,0)
        self.score = [0,0]

    def draw(self, x: int, y: int, w: int, h: int, color: int = 255):
        if y + h  > self.page_h:
            if y > self.page_h:
                raise RuntimeError("y out of range: {} > {}".format(y, self.page_h))  
            raise RuntimeError("y + h out of range: {} > {}".format(y + h, self.page_h)) 
        elif y - h < 0:
            if y < 0:
                raise RuntimeError("y out of range: {} < 0".format(y))  
            raise RuntimeError("y - h out of range: {} < 0".format(y- h)) 

        if x + w > self.page_w:

            if x > self.page_w:
                raise RuntimeError("x out of range: {} > {}".format(x, self.page_w))
            raise RuntimeError("x + w out of range: {} > {}".format(x + w, self.page_w))
        elif x - w < 0:
            # if x < 0:
                # raise RuntimeError("x out of range: {} < 0".format(x))  
            raise RuntimeError("x - w out of range: {} < 0".format(x - w)) 

        self.im[y-h:y+h, x-w:x+w] = [color,color,color]

    def draw_board(self, prevx, prevy, player):
        if player not in self.positions.keys():
            raise RuntimeError("Invlaid player: {}".format(player))
        x, y = self.positions[player]
        self.draw(floor(prevx), floor(prevy), self.board_w2 , self.board_h2, color=0)
        self.draw(floor(x), floor(y), self.board_w2 , self.board_h2, color=125)

    def draw_ball(self, prevx, prevy,):
        x,y = self.positions["b"]
        self.draw(floor(prevx), floor(prevy), self.ball_dim, self.ball_dim, color=0)
        self.draw(floor(x), floor(y), self.ball_dim, self.ball_dim)

    def start(self):
        self.positions["p1"] = (self.board_w2, (self.page_h - 2*self.board_h2)/2 + self.board_h2)
        self.draw_board(self.board_w2, (self.page_h - 2*self.board_h2)/2 + self.board_h2, "p1")
        self.positions["p2"] = (self.page_w - self.board_w2, (self.page_h - self.board_h2)/2 + self.board_h2)
        self.draw_board(self.page_w - self.board_w2, (self.page_h - self.board_h2)/2 + self.board_h2,"p2")
        self.positions["b"] = ((self.page_w)/2, (self.page_h)/2)
        self.draw_ball((self.page_w)/2, (self.page_h)/2)
        self.velocity = (
            ((-1) ** randint(1,2)) * randint(floor(self.page_w/20), floor(self.page_w/10)), 
            ((-1) ** randint(1,2)) * randint(floor(self.page_h/20), floor(self.page_h/10))
        )

    def setup(self):
        self.start()
        cv2.namedWindow("pong", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("pong", self.im)
        cv2.waitKey(3)

    def check_hit_board(self):
        x1, y1, vx1, vy1 = 0, 0, 0, 0
        x,y = self.positions["b"]
        p1x, p1y = self.positions["p1"]
        p2x, p2y = self.positions["p2"]
        prevx, prevy = x, y
        vx, vy = self.velocity
        x,y = floor(x + vx), floor(y + vy)

        # Check if ball is left of board 1
        if x - self.ball_dim <= p1x + self.board_w2:
            # Check that ball isnt above/ below board
            if y - self.ball_dim <= p1y + self.board_h2 and y + self.ball_dim >= p1y - self.board_h2:
                
                #hit from front
                x1, y1 = (
                    self.ball_dim + p1x + self.board_w2, 
                    # y + (tan(angle) * (board width - x))
                    floor(y + (tan(atan(vy/vx)) * (2*self.board_w2 + 1 - x)))
                )
                vx1, vy1 = -vx, vy

                # Board hit from bottom
                if y1 + self.ball_dim < p1y - self.board_h2:
                    print(y1)
                    print("bottom")
                    y1 = p1y + self.board_h2 + self.ball_dim
                    # x1 = x + dx, dx = dy / angle
                    x1 = x + ((-y1 + y) / tan(atan(vy/vx)))
                    vx1, vy1 = vx, -vy1
                    if x1 < self.ball_dim: return False
                    if x1 == self.ball_dim:
                        vx1, vy1 = -vx, -vy
                # board hit from top
                elif y1 - self.ball_dim > p1y + self.board_h2:
                    print(y1)
                    y1 = p1y - self.board_h2 - self.ball_dim
                    print("top")
                    # x1 = x + dx, dx = dy / angle
                    x1 = x + ((-y + y1) / tan(atan(vy/vx)))
                    vx1, vy1 = vx, -vy1
                    if x1 < self.ball_dim: return False
                    if x1 == self.ball_dim:
                        vx1, vy1 = -vx, -vy
                    
                print("x: {}, x1: {}, y: {}, y1: {}, vx: {}, vx1: {}, vy: {}, vy1: {}". format(x,x1,y,y1,vx,vx1,vy,vy1))
                self.positions["b"] = (x1, y1)
                self.velocity = (vx1, vy1)
                self.draw_ball(prevx, prevy)
                return True
        # Check if ball is right of board 2        
        elif x + self.ball_dim >= p2x - self.board_w2:
            # Check that ball isnt above/ below board
            if y - self.ball_dim <= p2y + self.board_h2 and y + self.ball_dim >= p2y - self.board_h2:
                #hit from front
                x1 = self.page_w - 2*self.board_w2 - 1 
                # y1 = y + (tan(angle) * (board width - x))
                y1 = floor(y + (tan(atan(vy/vx)) * (x - x1)))
                
                vx1, vy1 = -vx, vy

                # Board hit from bottom
                if y1 + self.ball_dim < p2y - self.board_h2:
                    print(y1)
                    y1 = p2y - self.board_h2 - self.ball_dim
                    print("top")
                    # x1 = x + dx, dx = dy / angle
                    x1 = x + ((-y + y1) / tan(atan(vy/vx)))
                    vx1, vy1 = vx, -vy1
                    if x1 < self.ball_dim: return False
                    if x1 == self.ball_dim:
                        vx1, vy1 = -vx, -vy
                # board hit from top
                elif y1 - self.ball_dim > p2y + self.board_h2:
                    print(y1)
                    print("bottom")
                    y1 = p2y + self.board_h2 + self.ball_dim
                    # x1 = x + dx, dx = dy / angle
                    x1 = x + ((-y1 + y) / tan(atan(vy/vx)))
                    vx1, vy1 = vx, -vy1
                    if x1 < self.ball_dim: return False
                    if x1 == self.ball_dim:
                        vx1, vy1 = -vx, -vy
                    
                print("x: {}, x1: {}, y: {}, y1: {}, vx: {}, vx1: {}, vy: {}, vy1: {}". format(x,x1,y,y1,vx,vx1,vy,vy1))
                self.positions["b"] = (x1, y1)
                self.velocity = (vx1, vy1)
                self.draw_ball(prevx, prevy)
                return True

    def move_ball(self):
        x1, y1, vx1, vy1 = 0, 0, 0, 0
        x,y = self.positions["b"]
        prevx, prevy = x, y
        vx, vy = self.velocity
        x,y = floor(x + vx), floor(y + vy)
        possible_score_l = False
        possible_score_r = False
        score = 0

        if self.check_hit_board(): 
            print("hit")
            return 0
               
        # left out of bounds
        if x - self.ball_dim <= 0:
            if x - self.ball_dim == 0:
                x1, y1 = x, y
                vx1, vy1 = -vx, vy
            else: 
                x1, y1 = self.ball_dim, floor(y + (tan(atan(vy/vx)) * (-x + self.ball_dim)))
                vx1, vy1 = -vx, vy
            possible_score_l = True
            
        # top out of bounds
        if y - self.ball_dim < 0:
            dx = (-y + self.ball_dim) / tan(atan(vy/vx))
            
            #Top-left edge case, check if new x is out of bounds
            if floor(x + dx) >= self.ball_dim and floor(x + dx) <= self.page_w - self.ball_dim:
                x1, y1 = floor(x + dx), self.ball_dim
                
                if floor(x + dx) > self.ball_dim and floor(x + dx) < self.page_w - self.ball_dim:
                    vx1, vy1 = vx, -vy
                else:
                    vx1, vy1 = -vx, -vy
            elif possible_score_l:
                score = 2
            else:
                possible_score_r = True
        elif y - self.ball_dim == 0:
            x1,y1 = x,y
            vx1, vy1 = vx, -vy
        
        # right out of bounds
        if x + self.ball_dim > self.page_w:
            dy = (tan(atan(vy/vx)) * (-x - self.ball_dim + self.page_w))
            
            #Top-right edge case
            if floor(y + dy) >= self.ball_dim and floor(y + dy) <= self.page_h - self.ball_dim:
                x1, y1 = self.page_w - self.ball_dim, floor(y + dy)
                
                if floor(y + dy) > self.ball_dim and floor(y + dy) < self.page_h - self.ball_dim:
                    vx1, vy1 = -vx, vy
                else:
                    vx1, vy1 = -vx, -vy
                possible_score_r = True
            elif possible_score_r:
                score = 1
        elif x + self.ball_dim == self.page_w:
            x1,y1 = x,y
            vx1, vy1 = -vx, vy
        
        # bottom out of bounds
        if y + self.ball_dim > self.page_h:
            dx = (-y - self.ball_dim + self.page_h) / tan(atan(vy/vx))

            #Bottom-right and bottom-left edge case
            if floor(x + dx) <= self.page_w - self.ball_dim and floor(x + dx) >= self.ball_dim:
                x1, y1 = floor(x + dx), self.page_h - self.ball_dim
                if x1 < self.page_w - self.ball_dim and x1 > self.ball_dim:
                    vx1, vy1 = vx, -vy
                else:
                    vx1, vy1 = -vx, -vy
                if x1 > self.page_w/2: score = 1
                else: score = 2
            elif possible_score_r: score = 1
            elif possible_score_l: score = 2
        elif y + self.ball_dim == self.page_h:
            x1,y1 = x, y
            vx1, vy1 = vx, -vy         
        
    

        if not x1 and not y1 and not vx1 and not vy1:
            x1, y1, vx1, vy1 = x, y, vx, vy
                    
        self.positions["b"] = (x1, y1)
        self.velocity = (vx1, vy1)
        self.draw_ball(prevx, prevy)
        return score


    def run(self):
        count = 0

        while count < 31:
            res = self.move_ball() 
            if res:
                self.score[res-1] += 1
                print("Player 1: {}, Player 2: {}".format(self.score[0], self.score[1]))
                x, y = self.positions["b"]
                self.draw_ball(x, y)
                self.start()
                time.sleep(3)           
            cv2.imshow("pong", self.im)
            cv2.waitKey(3)
            count += 1
            time.sleep(0.5)
        cv2.waitKey(0)
        cv2.destroyAllWindows()




p = Pong()
p.setup()
p.run()