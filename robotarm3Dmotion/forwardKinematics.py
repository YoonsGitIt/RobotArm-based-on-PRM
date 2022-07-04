import math
import numpy as np

d2r = math.pi/180
r2d = 180/math.pi

#point는 총 8개가 나올듯
#joint[5]는 엔드이펙터만 영향을 주는 거라서 포인트에서는 영향이 없다.
#ur5의 urdf를 보고 적용하였다.

def forwardkinematic(joint):
    
    joint = joint * d2r
    link = np.array([10,17,35,14,25,11,11,10])
    point1 = np.array([0,0,link[0]])
    point2 = np.array([link[1]*math.sin(joint[0]), link[1]*math.cos(joint[0]), point1[2]])
    point3 = np.array([point2[0] + link[2]*math.cos(joint[1]) , point2[1] , point2[2] + link[2]*math.sin(joint[1])])
    point4 = np.array([point3[0] - link[3]*math.sin(joint[0]) , point3[1]-link[3]*math.cos(joint[0]) , point3[2]])
    point5 = np.array([point4[0] + link[4]*math.cos(joint[2]) , point4[1] , point4[2] + link[4]*math.sin(joint[2])])
    point6 = np.array([point5[0] + link[5]*math.cos(joint[3]) , point5[1] , point5[2] + link[5]*math.sin(joint[3])])
    point7 = np.array([point6[0] , point6[1] , point6[2]-link[6]])
    point8 = np.array([point7[0] + link[7]*math.sin(joint[4]) , point7[1] , point7[2] + link[7]*math.cos(joint[4])])
    
    linkPoint = np.array([[0,0,0],point1,point2,point3,point4,point5,point6,point7,point8])

    return linkPoint