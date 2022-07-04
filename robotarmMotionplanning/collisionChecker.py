import math
import numpy as np
import matplotlib.pyplot as plt
import forwardKinematics as fk
#draw 
circle_center = np.array([[20,30],[50,50],[10,80],[50,-75],[-30,-70]])
circle_radius = np.array([10,15,20,20,10])
a = plt.axes(xlim=(-110, 130), ylim=(-110, 130))


for i in range(len(circle_radius)):
    c = plt.Circle(circle_center[i], circle_radius[i], fc='k', ec ='k')
    a.add_patch(c)
    a.set_aspect('equal') # X, Y축 1:1 비율로 설정

def collision_circle(center,radius,point):
    if (math.sqrt((center[0]-point[0])**2 + (center[1]-point[1])**2) <= (radius+10)):
        return True
    else:
        return False

def obstacleChecker(point):
    collision_count = 0
    for i in range(len(circle_radius)):
        if (collision_circle(circle_center[i],circle_radius[i],point)):
            collision_count += 1
    if(collision_count > 0):
        avoid = False
    else:
        avoid = True
        
    return avoid
    
def lineChecker(point1,point2):
    x_step = np.linspace(point1[0],point2[0],10)
    y_step = np.linspace(point1[1],point2[1],10)
    for i in range(1,9):
        if(obstacleChecker([x_step[i],y_step[i]])):
            continue
        else:
            return False
    return True

def checkJoint(joint):
    
    linkPoint = fk.forwardkinematic(joint)
    
    for i in range(len(linkPoint)-1):    
        if(lineChecker(linkPoint[i],linkPoint[i+1])):
            continue
        else:
            return False

    return True
