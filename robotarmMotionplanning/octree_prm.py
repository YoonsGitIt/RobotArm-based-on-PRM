import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def octree(map_size_x,map_size_y,map_size_z,coord_x,coord_y,coord_z,depth):
    area_list = np.empty(shape=0)
    for i in range(depth):
        x_center = (map_size_x[0] + map_size_x[1])/2
        y_center = (map_size_y[0] + map_size_y[1])/2
        z_center = (map_size_z[0] + map_size_z[1])/2

        if(coord_x < x_center and  coord_y >= y_center and coord_z >= z_center):
            map_size_x[1] = x_center
            map_size_y[0] = y_center
            map_size_z[0] = z_center
            area_list.append(area_list,[0,0,0],axis=0)
        elif(coord_x >= x_center and coord_y >= y_center and coord_z >= z_center):
            map_size_x[0] = x_center
            map_size_y[0] = y_center
            map_size_z[0] = z_center
            area_list.append(area_list,[0,1,0],axis=0)
        elif(coord_x < x_center and coord_y < y_center and coord_z >= z_center):
            map_size_x[1] = x_center
            map_size_y[1] = y_center
            map_size_z[0] = z_center
            area_list.append(area_list,[1,0,0],axis=0)
        elif(coord_x >= x_center and coord_y < y_center and coord_z >= z_center):
            map_size_x[0] = x_center
            map_size_y[1] = y_center
            map_size_z[0] = z_center
            area_list.append(area_list,[1,1,0],axis=0)
        elif(coord_x < x_center and  coord_y >= y_center and coord_z < z_center):
            map_size_x[1] = x_center
            map_size_y[0] = y_center
            map_size_z[1] = z_center
            area_list.append(area_list,[0,0,1],axis=0)
        elif(coord_x >= x_center and coord_y >= y_center and coord_z < z_center):
            map_size_x[0] = x_center
            map_size_y[0] = y_center
            map_size_z[1] = z_center
            area_list.append(area_list,[0,1,1],axis=0)
        elif(coord_x < x_center and coord_y < y_center and coord_z < z_center):
            map_size_x[1] = x_center
            map_size_y[1] = y_center
            map_size_z[1] = z_center
            area_list.append(area_list,[1,0,1],axis=0)
        elif(coord_x >= x_center and coord_y < y_center and coord_z < z_center):
            map_size_x[0] = x_center
            map_size_y[1] = y_center
            map_size_z[1] = z_center
            area_list.append(area_list,[1,1,1],axis=0)
    return area_list
