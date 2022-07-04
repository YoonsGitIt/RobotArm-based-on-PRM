import math
import numpy as np
import matplotlib.pyplot as plt

#draw obstacle
circle_center = np.array([[20,30],[35,80],[50,50],[80,20],[80,70]])
circle_radius = np.array([15,20,10,15,15])
a = plt.axes(xlim=(0, 110), ylim=(0, 110))
for i in range(len(circle_radius)):
    c = plt.Circle(circle_center[i], circle_radius[i], fc='k', ec ='k')
    a.add_patch(c)
    a.set_aspect('equal') # X, Y축 1:1 비율로 설정

def collision_circle(center,radius,point):
    if (math.sqrt((center[0]-point[0])**2 + (center[1]-point[1])**2) <= (radius+1)):
        return True
    else:
        return False

def avoid_obstacle(point):
    collision_count = 0
    for i in range(len(circle_radius)):
        if (collision_circle(circle_center[i],circle_radius[i],point[0])):
            collision_count += 1
    if(collision_count > 0):
        avoid = False
    else:
        avoid = True
    return avoid

def line_checker(point1,point2):
    x_step = np.linspace(point1[0],point2[0],10)
    y_step = np.linspace(point1[1],point2[1],10)
    for i in range(1,9):
        if(avoid_obstacle([[x_step[i],y_step[i]]])):
            continue
        else:
            checker = False
            return checker
    checker = True
    return checker