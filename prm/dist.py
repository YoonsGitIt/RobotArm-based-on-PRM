import math

def distance(point1,point2):
    dist = math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
    return dist