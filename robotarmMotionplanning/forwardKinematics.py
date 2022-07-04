import math
import numpy as np

d2r = math.pi/180
r2d = 180/math.pi

def forwardkinematic(joint):
    
    joint = joint * d2r
    link1 = 50
    link2 = 40
    link3 = 40
    point1 = np.array([link1*math.cos(joint[0]),link1*math.sin(joint[0])])
    point2 = np.array([point1[0]+link2*math.cos(joint[0]+joint[1]),point1[1]+link2*math.sin(joint[0]+joint[1])])
    point3 = np.array([point2[0]+link3*math.cos(joint[0]+joint[1]+joint[2]),point2[1]+link3*math.sin(joint[0]+joint[1]+joint[2])])
    
    linkPoint = np.array([[0,0],point1,point2,point3])

    return linkPoint