# -*- coding: utf-8 -*-
import time,requests
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from Kinetic import *
from numpy import *
import pygame

pygame.init()
ecran = pygame.display.set_mode((800, 800))
# ecran = pygame.display.set_mode()

k = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
print "Kinect lance"

# body1
yesyes = 0
hhead = 0
dhhead = 0
r_elbow = 0
l_elbow = 0
r_shoulder_x = 0
l_shoulder_x = 0
r_shoulder_y = 0
l_shoulder_y = 0
hrfoot = 0
hlfoot = 0
# body2
yesyes2 = 0
hhead2 = 0
dhhead2 = 0
r_elbow2 = 0
l_elbow2 = 0
r_shoulder_x2 = 0
l_shoulder_x2 = 0
r_shoulder_y2 = 0
l_shoulder_y2 = 0
hrfoot2 = 0
hlfoot2 = 0
# interbody
distneck = 0
disthand = 0
DHtete = 0
DXtete = 0
DYtete = 0
para_fore_arm = 0
continuer = True

while continuer :
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            continuer = False
    time.sleep(0.1)
    pos1 = False
    pos2 = False
    if k.has_new_body_frame():
        bs = k.get_last_body_frame()
        #print tiltrad*180.0/pi,w
        if bs is not None:
            for b in bs.bodies:
                if not b.is_tracked:
                    continue
                # get joints positions
                js = b.joints
                if pos1==False:
                    pos1 = extractPoints(js,0.0,0.0,0.0,0.0)
                elif pos2==False:
                    pos2 = extractPoints(js,0.0,0.0,0.0,0.0)

    if pos1!=False:
        # yes-yes angle
        head = pos1['neck']-pos1['head']
        spine = pos1['neck']-pos1['spine_base']
        yesyes = round(arccos(normalized_dot(head,spine))*180.0/pi)
        dhhead = (pos1['head'][2]-hhead)
        hhead = pos1['head'][2]
        # print("yesyes: "+str(yesyes))

        # right and left elbow angle
        right_arm = pos1['r_elbow']-pos1['r_shoulder']
        right_forearm = pos1['r_elbow']-pos1['r_wrist']
        left_arm = pos1['l_elbow']-pos1['l_shoulder']
        left_forearm = pos1['l_elbow']-pos1['l_wrist']
        r_elbow = round(arccos(normalized_dot(right_arm,right_forearm))*180.0/pi)
        l_elbow = round(arccos(normalized_dot(left_arm,left_forearm))*180.0/pi)
        # print("r_elbow: "+str(r_elbow))
        # print("l_elbow: "+str(l_elbow))

        # right and left shoulder x
        bust = pos1['r_shoulder']-pos1['l_shoulder']
        r_shoulder_x = round(arccos(normalized_dot(right_arm,-bust))*180.0/pi)
        l_shoulder_x = round(arccos(normalized_dot(left_arm,bust))*180.0/pi)
        # print("r_shoulder_x: "+str(r_shoulder_x))
        # print("l_shoulder_x: "+str(l_shoulder_x))

        # right and left shoulder y
        r_shoulder_y = round(arccos(normalized_dot(right_arm,spine))*180.0/pi)
        l_shoulder_y = round(arccos(normalized_dot(left_arm,spine))*180.0/pi)
        # print("r_shoulder_y: "+str(r_shoulder_y))
        # print("l_shoulder_y: "+str(l_shoulder_y))

        # feet
        hrfoot = pos1['r_ankle'][2]
        hlfoot = pos1['l_ankle'][2]

    if pos2!=False:
        # yes-yes angle
        head2 = pos2['neck']-pos2['head']
        spine2 = pos2['neck']-pos2['spine_base']
        yesyes2 = round(arccos(normalized_dot(head2,spine2))*180.0/pi)
        hhead2 = pos2['head'][2]
        dhhead2 = (pos2['head'][2]-hhead2)
        # print("yesyes: "+str(yesyes))

        # right and left elbow angle
        right_arm2 = pos2['r_elbow']-pos2['r_shoulder']
        right_forearm2 = pos2['r_elbow']-pos2['r_wrist']
        left_arm2 = pos2['l_elbow']-pos2['l_shoulder']
        left_forearm2 = pos2['l_elbow']-pos2['l_wrist']
        r_elbow2 = round(arccos(normalized_dot(right_arm2,right_forearm2))*180.0/pi)
        l_elbow2 = round(arccos(normalized_dot(left_arm2,left_forearm2))*180.0/pi)
        # print("r_elbow: "+str(r_elbow))
        # print("l_elbow: "+str(l_elbow))

        # right and left shoulder x
        bust2 = pos2['r_shoulder']-pos2['l_shoulder']
        r_shoulder_x2 = round(arccos(normalized_dot(right_arm2,-bust2))*180.0/pi)
        l_shoulder_x2 = round(arccos(normalized_dot(left_arm2,bust2))*180.0/pi)
        # print("r_shoulder_x: "+str(r_shoulder_x))
        # print("l_shoulder_x: "+str(l_shoulder_x))

        # right and left shoulder y
        r_shoulder_y2 = round(arccos(normalized_dot(right_arm2,spine2))*180.0/pi)
        l_shoulder_y2 = round(arccos(normalized_dot(left_arm2,spine2))*180.0/pi)
        # print("r_shoulder_y: "+str(r_shoulder_y2))
        # print("l_shoulder_y: "+str(l_shoulder_y2))

        # feet
        hrfoot2 = pos2['r_ankle'][2]
        hlfoot2 = pos2['l_ankle'][2]

    if pos1!=False and pos2!=False:
        # distance entre les nuques
        distneck = norm(pos2['neck']-pos1['neck'])

        # distance entre les mains
        disthand = norm(pos2['r_wrist']-pos1['r_wrist'])

        # delta altitude tetes
        DHtete = pos2['neck'][2]-pos1['neck'][2]

        # delta profondeur tetes
        DXtete = pos2['neck'][2]-pos1['neck'][2]

        # delta symetrie tete
        DYtete = pos2['neck'][1]+pos1['neck'][1]

        # avant bras parallele
        para_fore_arm = normalized_cross_prod(right_forearm2,right_forearm)


    cost=0
    mod = 7
    # SOLO
    # symetrie des coudes
    if mod==1:
        cost += min(abs((round(l_elbow)-round(r_elbow))/1),100)

    # symetrie des epaules
    if mod==2:
        cost += min(abs((round(l_shoulder_x)-round(r_shoulder_x))/1),100)

    # altitude pied droit
    if mod==3:
        print("hrfoot:")
        print(hrfoot)
        cost += min(abs(hrfoot)*100,100)

    # altitude tete
    if mod==4:
        print("hhead:")
        print(hhead)
        cost += min(abs(hhead)*100,100)

    # DUO
    # distanciation sociale nuque
    if mod==5:
        print("distneck:")
        print(distneck)
        cost += min(abs(distneck-2.0)*100,100)

    # distanciation sociale main
    if mod==6:
        print("disthand:")
        print(disthand)
        cost += min(abs(disthand-1.0)*100,100)

    # gestion de taille
    if mod==7:
        print("DHtete:")
        print(DHtete)
        cost += min(abs(DHtete-0)*100,100)

    # gestion de profondeur
    if mod==8:
        print("DXtete:")
        print(DXtete)
        cost += min(abs(DXtete-0)*100,100)

    # parallelisme d'avant bras
    if mod==9:
        print("para_fore_arm:")
        print(para_fore_arm)
        cost += min(abs(para_fore_arm)*100,100)

    # gestion de profondeur
    if mod==8:
        print("DYtete:")
        print(DYtete)
        cost += min(abs(DYtete-0)*100,100)
    
    # symetrie Hpied
    if mod==10:
        print("hrfoot-hlfoot:")
        print(hrfoot-hlfoot)
        cost += min(abs(hrfoot-hlfoot)*100,100)

    # mouvement tete
    if mod==11:
        print("dhhead")
        print(dhhead)
        cost += min(abs(dhhead)*10000,100)

    # try:
    #r = requests.get('http://10.0.0.123/win&A=10&IX='+str(cost)+'&PL=4')
    # except:
        # pass
    print(cost)
    grey = cost*255/100
    ecran.fill((grey,grey,grey))
    pygame.display.flip()


pygame.quit()
                