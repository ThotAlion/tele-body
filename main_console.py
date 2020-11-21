# -*- coding: utf-8 -*-
import time,requests
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from Kinetic import *
from numpy import *
#import pyttsx



k = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
print "Kinect lance"

while True :
    # time.sleep(0.1)
    if k.has_new_body_frame():
        bs = k.get_last_body_frame()
        #print tiltrad*180.0/pi,w
        if bs is not None:
            for b in bs.bodies:
                if not b.is_tracked:
                    continue
                # get joints positions
                js = b.joints
                pos = extractPoints(js,0.0,0.0,0.0,0.0)

                # yes-yes angle
                head = pos['neck']-pos['head']
                spine = pos['neck']-pos['spine_base']
                yesyes = round(arccos(normalized_dot(head,spine))*180.0/pi)
                # print("yesyes: "+str(yesyes))

                # right and left elbow angle
                right_arm = pos['r_elbow']-pos['r_shoulder']
                right_forearm = pos['r_elbow']-pos['r_wrist']
                left_arm = pos['l_elbow']-pos['l_shoulder']
                left_forearm = pos['l_elbow']-pos['l_wrist']
                r_elbow = round(arccos(normalized_dot(right_arm,right_forearm))*180.0/pi)
                l_elbow = round(arccos(normalized_dot(left_arm,left_forearm))*180.0/pi)
                # print("r_elbow: "+str(r_elbow))
                # print("l_elbow: "+str(l_elbow))

                # right and left shoulder x
                bust = pos['r_shoulder']-pos['l_shoulder']
                r_shoulder_x = round(arccos(normalized_dot(right_arm,-bust))*180.0/pi)
                l_shoulder_x = round(arccos(normalized_dot(left_arm,bust))*180.0/pi)
                # print("r_shoulder_x: "+str(r_shoulder_x))
                # print("l_shoulder_x: "+str(l_shoulder_x))

                # right and left shoulder y
                r_shoulder_y = round(arccos(normalized_dot(right_arm,spine))*180.0/pi)
                l_shoulder_y = round(arccos(normalized_dot(left_arm,spine))*180.0/pi)
                print("r_shoulder_y: "+str(r_shoulder_y))
                print("l_shoulder_y: "+str(l_shoulder_y))

                cost=0

                # symetrie des coudes
                cost += min(abs((round(l_elbow)-round(r_elbow))/1),100)

                # symetrie des epaules
                # cost += min(abs((round(l_shoulder_x)-round(r_shoulder_x))/1),100)
                r = requests.get('http://10.0.0.123/win&A=10&IX='+str(cost)+'&PL=4')



                