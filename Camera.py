#! /usr/bin/python3

import rospy
import time
import cv2
import numpy as np
from utils import filter_img, find_centroids, sort_points, connect_points
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

DIMENSION = (720,960) #Height, Width

class TakePhoto:
    def __init__(self):
        self.image_received = False

        img_topic = "/camera/image_rect_color/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback, queue_size = 1)

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    def get_img(self):
        dim = (960,720)
        return cv2.resize(self.img, dim)
    def save_img(self, img_title):
        print(cv2.imwrite(img_title, self.img))

class CmdListener:
    def __init__(self):
        self.cmd = ""
        self.cmd_sub = rospy.Subscriber("/master_cmd", String, self.callback)
    def callback(self, cmd):
        self.cmd = cmd.data
    def get_msg(self):
        return self.cmd

def get_frames_in_batches_on_off(camera_obj, cmd_listener, batch_size = 1):
    on_imgs = np.zeros((batch_size, DIMENSION[0], DIMENSION[1]))
    off_imgs = np.zeros((batch_size, DIMENSION[0], DIMENSION[1]))
    for i in range(batch_size):
        if cmd_listener.get_msg() == "HIGH":
            time.sleep(0.45)
            on_imgs[i] = cv2.cvtColor(camera.get_img(), cv2.COLOR_BGR2GRAY)

            while cmd_listener.get_msg() == "HIGH":
                pass
        if cmd_listener.get_msg() == "LOW":
            time.sleep(0.45) 
            off_imgs[i] = cv2.cvtColor(camera.get_img(), cv2.COLOR_BGR2GRAY)
            
            while cmd_listener.get_msg() == "LOW":
                pass
    return on_imgs, off_imgs

if __name__ == "__main__":
    i = 0
    rospy.init_node("CameraProcessing")
    camera = TakePhoto()
    cmd_listener = CmdListener()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        while True:
            start_timepoint = time.time()
            
            on_imgs, off_imgs= get_frames_in_batches_on_off(camera, cmd_listener, 1)

            on_img, off_img, binary_difference = filter_img(on_imgs, off_imgs)
            cv2.imwrite("/home/robot149a/resulted_img/binary_difference.jpg", binary_difference*255)

            cnts, _ = cv2.findContours(image = binary_difference.astype(np.uint8), mode = cv2.RETR_EXTERNAL, method = cv2.CHAIN_APPROX_SIMPLE)

            centroids = find_centroids(cnts, binary_difference)
            
            print("Can currently see " + str(len(centroids)) + " leds.")
            if (len(centroids)) == 3:
                print("See leds")
                print("---")
                break
            else:
                print("Cannot see leds properly. Please wait!")
        centroids = sort_points(centroids)
         
        print("Left front led's coordinate: " + str(centroids[0]))
        print("Right front led's coordinate: " + str(centroids[1]))
        print("Back led's coordinate: " + str(centroids[2]))
        
        
        connect_points(on_img, centroids)
        cv2.imwrite("/home/robot149a/resulted_img/leds_connect.jpg", on_img)
        
        print("Total time to find the leds: " + str(time.time() - start_timepoint))
        print("------------------------------------------------------------")
        
        time.sleep(0.5)
        i+=1
      
        rate.sleep()
