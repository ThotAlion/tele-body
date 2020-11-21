# -*- coding: utf-8 -*-
import time
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from Kinetic import extractPoints
from numpy import *
#import pyttsx



k = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
print "Kinect lance"
#e = pyttsx.init()

#e.say('Bonjour et bienvenu dans la prossaidure de calibration de la machine vivante. Une personne doit se mettre debout au centre de la saine, face public, les bras ecartai comme jaizu cri. et une autre personne est praite a tourner la Kinect selon l''axe Z. Tenez vous prai dans dix, neuf, huit, sept, six, cinq, quatre, trois, deux, un.')
#e.runAndWait()
calib = True
while calib :
    time.sleep(0.1)
    seeBody = False
    if k.has_new_body_frame():
        bs = k.get_last_body_frame()
        tiltrad = arctan(bs.floor_clip_plane.z/bs.floor_clip_plane.y)
        w = bs.floor_clip_plane.w
        #print tiltrad*180.0/pi,w
        if bs is not None:
            for b in bs.bodies:
                if not b.is_tracked:
                    continue
                # get joints positions
                js = b.joints
                kpos = extractPoints(js,tiltrad,w,0.0,0.0)
                if kpos["spine_base"][1]>0.05:
                    # e.say(u'tourner la kinect un peu a droite!')
                    # e.runAndWait()
                    print(u'tourner la kinect un peu a droite!')
                elif kpos["spine_base"][1]<-0.05:
                    # e.say(u'tourner la kinect un peu a gauche!')
                    # e.runAndWait()
                    print(u'tourner la kinect un peu a gauche!')
                else:
                    # e.say('c''est bon ne touchez plus la Kinect, tout est calibrai. Merci de votre devoumain')
                    # e.runAndWait()
                    print('c''est bon ne touchez plus la Kinect, tout est calibrai. Merci de votre devoumain')
                    print "rtip"
                    print kpos["r_tip"]
                    print "ltip"
                    print kpos["l_tip"]
                    print "spine"
                    print kpos["spine_base"]
                    print "tilt"
                    print tiltrad*180.0/pi
                    print "hkinect"
                    print w
                    print "dkinect"
                    print -kpos["spine_base"][0]
                    print "pan"
                    print arctan((kpos["r_tip"][0]-kpos["l_tip"][0])/(kpos["r_tip"][1]-kpos["l_tip"][1]))*180.0/pi
                    calib = False