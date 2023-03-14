#!/usr/bin/python3

import rospy
import time
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class TakePhoto:
    def __init__(self):
        self.image_received = False

        img_topic = "/raspicam_node/image/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback, queue_size = 1)

    def callback(self, data):
        self.image_received = True
        np_arr = np.fromstring(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    def get_img(self):
        if self.image_received == False:
           print("None")
           return
        return self.img
    def save_img(self, img_title):
        if self.image_received == False:
            print("None")
            return
        cv2.imwrite(img_title, self.img)

class CmdListener:
    def __init__(self):
        self.cmd = ""
        self.cmd_sub = rospy.Subscriber("/master_cmd", String, self.callback)
    def callback(self, cmd):
        self.cmd = cmd.data
    def get_msg(self):
        return self.cmd

if __name__ == "__main__":
    i = 0
    rospy.init_node("CameraProcessing")
    camera = TakePhoto()
    cmd_listener = CmdListener()
    rate = rospy.Rate(10)
    led_on = None
    led_off = None
    time.sleep(3)
    while not rospy.is_shutdown():
        if cmd_listener.get_msg() == "HIGH":
            time.sleep(0.45)
            led_on = camera.get_img()
            camera.save_img("/home/parallels/led_on/photo{}.jpg".format(i))
            while cmd_listener.get_msg() == "HIGH":
                pass
        if cmd_listener.get_msg() == "LOW":
            time.sleep(0.45)
            led_off = camera.get_img()
            camera.save_img("/home/parallels/led_off/photo{}.jpg".format(i))
            while cmd_listener.get_msg() == "LOW":
                pass
        if (led_on is not  None and led_off is not None):
            diff = cv2.absdiff(led_on, led_off)
            cv2.imwrite("/home/parallels/led_diff/photo{}.jpg".format(i), diff)
            print(i)
            i+=1
        rate.sleep()

