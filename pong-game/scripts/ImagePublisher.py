import cv_bridge
import rospy
import intera_interface
from sensor_msgs.msg import Image

class ImagePublisher(object):

    def __init__(self,refreshRate):
        self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        rospy.init_node('PongGame',anonymous=True,disable_signals=True)
        self.__r = rospy.Rate(refreshRate)

    def publishImage(self, img):
        imageMsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="8UC3")
        #if not rospy.is_shutdown():
        self._image_pub.publish(imageMsg)
        self.__r.sleep() 