env = "Sawyer"
from time import sleep
import numpy as np
import cv2
from Settings import Settings
try:
    from cv_bridge import CvBridge, CvBridgeError
    import rospy
    import intera_interface
    from sensor_msgs.msg import Image
except ModuleNotFoundError:
    env = "PC"

class Window(Settings):
    def __init__(self):
        Settings.__init__(self)
        self.image = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8) 
        
        if env == "Sawyer":
            self.image_pub = rospy.Publisher("/robot/head_display", Image, latch=True, queue_size=10)
            self.display = intera_interface.HeadDisplay()

    def _show_image_pc(self):
        # Show image in window
        cv2.imshow("Pong", self.image)
        sleep(1/60)
        key = cv2.waitKey(3)
        if key != -1:
            if chr(key) == "q":
                return False
            return key
        return True

    def _show_image_sawyer(self):
        # Show image on display
        if rospy.is_shutdown():
            return False
        bridge = CvBridge()
        img = bridge.cv2_to_imgmsg(self.image, encoding="8UC3")
        self.image_pub.publish(img)
        rospy.Rate(60).sleep()
        return True

    def display_image(self):
        if env == "Sawyer":
            return self._show_image_sawyer()
        return self._show_image_pc()
    
    def close_image(self, wait = 0):
        cv2.waitKey(wait)
        cv2.destroyAllWindows()

    def draw_right_player(self, position_y):
        cv2.rectangle(
            self.image,
            (self.window_width - self.bar_width, int(position_y - self.bar_height/2)),
            (self.window_width - 1, int(position_y + self.bar_height/2)),
            (255, 255, 255),
            -1
        )

    def draw_left_player(self, position_y):
        cv2.rectangle(
            self.image,
            (0, int(position_y - (self.bar_height - 1)/2)),
            (self.bar_width - 1, int(position_y + (self.bar_height - 1)/2)),
            (255, 255, 255),
            -1
        )

    def draw_ball(self, position_x, position_y):
        #cv2.rectangle(
        #    self.image,
        #    (int(position_x - (self.ball_dimension - 1)/2), int(position_y - (self.ball_dimension - 1)/2)),
        #    (int(position_x + (self.ball_dimension - 1)/2), int(position_y + (self.ball_dimension - 1)/2)),
        #    (255, 255, 255),
        #    -1
        #)
        cv2.circle(
            self.image, 
            (int(position_x), int(position_y)), 
            (self.ball_dimension-1)/2, 
            (255, 255, 255), 
            -1
        )

    def draw_score(self, score_left, score_right):
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
