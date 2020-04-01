#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path

import collections
import threading
import numpy as np
import cv2

class DisplayImage(threading.Thread):
    def __init__(self, map_gen):
        threading.Thread.__init__(self)
        self.map_gen = map_gen
    
    def run(self):
        while True:
            cv2.imshow('Map', self.map_gen.map_img)
            if cv2.waitKey(10) & 0xff == ord('q'):
                break

class MapGenerator():
    def __init__(self):
        self.scale_rate = 75
        self.bias = (50, 430)
        self.map_img = np.zeros((512, 1024, 3), np.uint8)
        self.map_img.fill(255)
        self.previous_point = None
        self.pre_points = collections.deque(maxlen=10)
        #cv2.imshow('Map', self.map_img)
        #cv2.waitKey(0)

    def draw_path(self, new_point):
        if self.previous_point:
            cv2.line(self.map_img, self.previous_point, new_point, (200, 200, 200), 15)
        self.previous_point = new_point
        self.pre_points.append(new_point)
        for i in range(1, len(self.pre_points)):
            if self.pre_points[i-1] is None or self.pre_points[i] is None:
                continue
            cv2.line(self.map_img, self.pre_points[i-1], self.pre_points[i], (20, 255, 20), 2)
        cv2.circle(self.map_img, new_point, 3, (0, 0, 255), -1)
        #cv2.imshow('Map', self.map_img)

    def show_img(self):
        while True:
            cv2.imshow('Map', self.map_img)
            if cv2.waitKey(10) & 0xff == ord('q'):
                break

    def callback(self, data):
        pos = data.poses[-1].pose.position
        rospy.loginfo(rospy.get_caller_id() + '(%s, %s)', pos.x, pos.y)
        img_point = (int(pos.x * self.scale_rate + self.bias[0]), int(pos.y * self.scale_rate + self.bias[1]))
        self.draw_path(img_point)

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/loop_fusion/pose_graph_path', Path, self.callback)
        rospy.spin()

if __name__ == '__main__':
    map_gernerator = MapGenerator()
    display_thread = DisplayImage(map_gernerator)
    display_thread.start()
    map_gernerator.listener()
    print("Listener closed")
    display_thread.join()
    print("Done!")
   # map_gernerator.show_img()

