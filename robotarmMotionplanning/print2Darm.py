import matplotlib.pyplot as plt

def printRobotarm(point):
    point1=point[1]
    point2=point[2]
    point3=point[3]
    plt.plot([0,point1[0]],[0,point1[1]],color='c',linewidth='5')
    plt.plot([point1[0],point2[0]],[point1[1],point2[1]],color='g',linewidth='5')
    plt.plot([point2[0],point3[0]],[point2[1],point3[1]],color='k',linewidth='5')