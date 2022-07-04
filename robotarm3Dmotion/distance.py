from cmath import sqrt
import math
import numpy as np

def distance(point1,point2):
    distance_square = 0
    for i in range(len(point1)):
        distance_square = distance_square + (point1[i]-point2[i])**2
        
    distance = math.sqrt(distance_square)
    return distance
