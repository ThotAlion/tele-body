from numpy import *
from numpy.linalg import *
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

def cross_prod(a,b):
    ax = array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return dot(ax,b)
    
def normalized_cross_prod(a,b):
    return cross_prod(a,b)/(norm(cross_prod(a,b)))
    
def normalized_dot(a,b):
    return dot(a,b)/(norm(a)*norm(b))

def toVect(joint,tilt,w,pan,d):
    angle_cam = tilt
    t=rot_mat('y',sin(tilt),cos(tilt))
    p=rot_mat('z',sin(pan),cos(pan))
    #if joint.TrackingState == PyKinectV2.TrackingState_Tracked:
    tt = dot(p,dot(t,array([-joint.Position.z+d,-joint.Position.x,joint.Position.y])))
    tt[2] = tt[2]+w
    return tt
    #else:
    #    return array([0.0,0.0,4.0])

def rot_mat(axe,s,c):
    if axe == "x":
        return array([[1,0,0],[0,c,-s],[0,s,c]])
    elif axe == "y":
        return array([[c,0,s],[0,1,0],[-s,0,c]])
    elif axe == "z":
        return array([[c,-s,0],[s,c,0],[0,0,1]])
    else:
        return 0
        
def extractPoints(js,tilt,w,pan,d):
    
    pos = {}
    # mirrored
    pos['l_shoulder'] = toVect(js[PyKinectV2.JointType_ShoulderRight],tilt,w,pan,d)
    pos['l_elbow'] = toVect(js[PyKinectV2.JointType_ElbowRight],tilt,w,pan,d)
    pos['l_wrist'] = toVect(js[PyKinectV2.JointType_WristRight],tilt,w,pan,d)
    pos['l_tip'] = toVect(js[PyKinectV2.JointType_HandTipRight],tilt,w,pan,d)
    
    pos['l_hip'] = toVect(js[PyKinectV2.JointType_HipRight],tilt,w,pan,d)
    pos['l_knee'] = toVect(js[PyKinectV2.JointType_KneeRight],tilt,w,pan,d)
    pos['l_ankle'] = toVect(js[PyKinectV2.JointType_AnkleRight],tilt,w,pan,d)
    pos['l_toe'] = toVect(js[PyKinectV2.JointType_FootRight],tilt,w,pan,d)
    
    pos['r_shoulder'] = toVect(js[PyKinectV2.JointType_ShoulderLeft],tilt,w,pan,d)
    pos['r_elbow'] = toVect(js[PyKinectV2.JointType_ElbowLeft],tilt,w,pan,d)
    pos['r_wrist'] = toVect(js[PyKinectV2.JointType_WristLeft],tilt,w,pan,d)
    pos['r_tip'] = toVect(js[PyKinectV2.JointType_HandTipLeft],tilt,w,pan,d)

    pos['r_hip'] = toVect(js[PyKinectV2.JointType_HipLeft],tilt,w,pan,d)
    pos['r_knee'] = toVect(js[PyKinectV2.JointType_KneeLeft],tilt,w,pan,d)
    pos['r_ankle'] = toVect(js[PyKinectV2.JointType_AnkleLeft],tilt,w,pan,d)
    pos['r_toe'] = toVect(js[PyKinectV2.JointType_FootLeft],tilt,w,pan,d)
    
    pos['spine_base'] = toVect(js[PyKinectV2.JointType_SpineBase],tilt,w,pan,d)
    pos['head'] = toVect(js[PyKinectV2.JointType_Head],tilt,w,pan,d)
    pos['neck'] = toVect(js[PyKinectV2.JointType_Neck],tilt,w,pan,d)
    
    return pos