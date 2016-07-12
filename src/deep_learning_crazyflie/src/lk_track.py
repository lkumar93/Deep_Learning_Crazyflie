#!/usr/bin/env python

'''
Lucas-Kanade tracker
====================

Lucas-Kanade sparse optical flow demo. Uses goodFeaturesToTrack
for track initialization and back-tracking for match verification
between frames.

Usage
-----
lk_track.py [<video_source>]


Keys
----
ESC - exit
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import numpy
import cv2
import video
import math
from common import anorm2, draw_str
from time import clock
from mavros_msgs.msg import OpticalFlowRad
import rospy

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

class App:
    def __init__(self, video_src):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.cam = video.create_capture(video_src)
        self.frame_idx = 0
	self.DistanceTravelledX = 0
	self.DistanceTravelledY = 0
	self.pub = rospy.Publisher('OpticalFlowXY', OpticalFlowRad, queue_size=10)
	self.msg = OpticalFlowRad()
	rospy.init_node('OpticalFlowXYNode')
	self.rate = rospy.Rate(1000)

    def run(self):
        while True:
	    TickCountBefore = cv2.getTickCount()
            ret, frame = self.cam.read()
	    TimePeriod = 1/cv2.getTickFrequency()
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	    
	    dst = cv2.medianBlur(frame_gray, 5)
 	    abc = dst
	    frame_gray = dst

            vis = frame.copy()
	    
            VelocityX = 0
	    VelocityY = 0

            if len(self.tracks) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
                
                p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)

		
                p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
                d = abs(p0-p0r).reshape(-1, 2).max(-1)
                good = d < 1
                new_tracks = []
		
		index = 0;
		distance = 10000;

		
		

                for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                    if not good_flag:
                        continue
                    tr.append((x, y))
                    if len(tr) > self.track_len:
                        del tr[0]
                    new_tracks.append(tr)
                    cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
		    cv2.circle(abc, (x, y), 2, (0, 255, 0), -1)

		for i in range(0,len(self.tracks)):
		    localdistance = math.sqrt(math.pow((p0.item(i)-p0r.item(i)),2) + math.pow((p0.item(i+1)-p0r.item(i+1)),2)) 
		    if localdistance < distance :
			distance = localdistance
			index = i
		
		TickCountAfter = cv2.getTickCount()
	        
		TimeElapsed = (TickCountAfter-TickCountBefore)*TimePeriod
	
		draw_str(vis, (20, 110), 'Time Elapsed  %f' % TimeElapsed)
		draw_str(vis, (20, 130), 'TimePeriod  %f' % TimePeriod)
		
		#VelocityX = (Average_X_Velocity_P1 - Average_X_Velocity_P0)/(TimeElapsed *379.05)
		#VelocityY = (Average_Y_Velocity_P1 - Average_Y_Velocity_P0)/(TimeElapsed *366.6)
		
		VelocityX = (p1.item(index) - p0.item(index))*100 /(TimeElapsed *379.05)
		VelocityY = (p1.item(index+1) - p0.item(index+1))*100/(TimeElapsed*366.6)
		self.msg.integrated_x = VelocityX;
		self.msg.integrated_y = VelocityY;
		self.msg.integration_time_us = TimeElapsed*1000000;	
		self.msg.header.stamp = rospy.Time.now()
		self.pub.publish(self.msg)
		self.rate.sleep()

		self.DistanceTravelledX = self.DistanceTravelledX + (VelocityX*TimeElapsed)
		self.DistanceTravelledY = self.DistanceTravelledY + (VelocityY*TimeElapsed)
		
		
                index1 = " Item 1  x" + str(p1.item(index)) + " " + str(p1.item(index+1))
		index2 = " Item 0 x " + str(p0.item(index)) + " " + str(p0.item(index+1))
		index3 = " Item 0r x " + str(p0r.item(index)) + " " + str(p0r.item(index+1))

		print(index1)
		print(index2)
		print(index3)
		
                self.tracks = new_tracks
                cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))

		cv2.polylines(abc, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))
               
		draw_str(vis, (20, 50), 'Distance x %f' % self.DistanceTravelledX)
		draw_str(vis, (20, 80), 'Distance y %f' % self.DistanceTravelledY)
		#draw_str(vis, (20, 50), 'Velocity x %f' % VelocityX)
		#draw_str(vis, (20, 80), 'Velocity y: %f' % VelocityY)
		
		

            if self.frame_idx % self.detect_interval == 0:
                mask = np.zeros_like(frame_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                    cv2.circle(mask, (x, y), 5, 0, -1)
                p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                if p is not None:
                    for x, y in np.float32(p).reshape(-1, 2):
                        self.tracks.append([(x, y)])


            self.frame_idx += 1
            self.prev_gray = frame_gray
            cv2.imshow('lk_track', vis)
	    cv2.imshow('lk_track2', abc)

            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                break

def main():
    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 0

    print(__doc__)
    App(video_src).run()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
