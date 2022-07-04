import math
import numpy as np
import matplotlib.pyplot as plt
import forwardKinematics as fk
import distance as dist
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

#circle
circle_center = np.array([[10,30,80],[50,70,40]])
circle_radius = np.array([20,20])

##########  collision check  ##############

def collision_circle(center,radius,point):
    if (dist.distance(center,point) <= (radius+5)):
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
    split_size = 10
    x_step = np.linspace(point1[0],point2[0],split_size)
    y_step = np.linspace(point1[1],point2[1],split_size)
    z_step = np.linspace(point1[1],point2[1],split_size)
    for i in range(1,split_size-1):
        if(obstacleChecker([x_step[i],y_step[i],z_step[i]])):
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