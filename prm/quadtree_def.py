import matplotlib.pyplot as plt
import numpy as np

def quadtree(map_size_x,map_size_y,coord_x,coord_y,depth):
    area_list = np.empty(shape=0)
    row_value = 0
    colum_value = 0
    for i in range(depth):       
        x_center = (map_size_x[0] + map_size_x[1])/2
        y_center = (map_size_y[0] + map_size_y[1])/2

        if(coord_x < x_center and  coord_y >= y_center):
            map_size_x[1] = x_center
            map_size_y[0] = y_center
            area_list = np.append(area_list,[0,0],axis=0)
        elif(coord_x >= x_center and coord_y >= y_center):
            map_size_x[0] = x_center
            map_size_y[0] = y_center
            area_list = np.append(area_list,[0,1],axis=0)
        elif(coord_x < x_center and coord_y < y_center):
            map_size_x[1] = x_center
            map_size_y[1] = y_center
            area_list = np.append(area_list,[1,0],axis=0)
        elif(coord_x >= x_center and coord_y < y_center):
            map_size_x[0] = x_center
            map_size_y[1] = y_center
            area_list = np.append(area_list,[1,1],axis=0)
                        
        colum_value = colum_value + (2**(depth-i-1))*area_list[2*i]
        row_value = row_value + (2**(depth-i-1))*area_list[2*i+1]
                
    array_val = np.array([colum_value ,row_value],int)
    return array_val