import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

map_size_x = [-100,100]
map_size_y = [-100,100]
map_size_z = [-100,100]

coord_x = float(input("Input x coordinate : "))
coord_y = float(input("Input y coordinate : "))
coord_z = float(input("Input y coordinate : "))
depth = int(input("Input depth : "))

fig = plt.figure(figsize=(9, 6))
ax = fig.add_subplot(111, projection='3d')

def check_area (size_x,size_y,size_z,x,y,z):
    
    x_center = float(size_x[0] + size_x[1])/2
    y_center = float(size_y[0] + size_y[1])/2
    z_center = float(size_z[0] + size_z[1])/2

    if (x < x_center and  y >= y_center and z >= z_center):
        area = 1
    elif (x >= x_center and y >= y_center and z >= z_center):
        area = 2
    elif (x < x_center and y < y_center and z >= z_center):
        area = 3
    elif (x >= x_center and y < y_center and z >= z_center):
        area = 4
    elif (x < x_center and  y >= y_center and z < z_center):
        area = 5
    elif (x >= x_center and y >= y_center and z < z_center):
        area = 6
    elif (x < x_center and y < y_center and z < z_center):
        area = 7
    elif (x >= x_center and y < y_center and z < z_center):
        area = 8
    return area

def octreeList(map_size_x,map_size_y,map_size_z,coord_x,coord_y,coord_z,depth):
    area_list = []
    for i in range(depth):
        x_center = (map_size_x[0] + map_size_x[1])/2
        y_center = (map_size_y[0] + map_size_y[1])/2
        z_center = (map_size_z[0] + map_size_z[1])/2
        ax.plot([map_size_x[0],map_size_x[1]],[y_center,y_center],[z_center,z_center],color = "black")
        ax.plot([x_center,x_center],[map_size_y[0],map_size_y[1]],[z_center,z_center],color = "black")
        ax.plot([x_center,x_center],[y_center,y_center],[map_size_z[0],map_size_z[1]],color = "black")

        current_area = check_area(map_size_x,map_size_y,map_size_z,coord_x,coord_y,coord_z)
        if(current_area == 1):
            map_size_x[1] = x_center
            map_size_y[0] = y_center
            map_size_z[0] = z_center
            area_list.append("nwu")
        elif(current_area == 2):
            map_size_x[0] = x_center
            map_size_y[0] = y_center
            map_size_z[0] = z_center
            area_list.append("neu")
        elif(current_area == 3):
            map_size_x[1] = x_center
            map_size_y[1] = y_center
            map_size_z[0] = z_center
            area_list.append("swu")
        elif(current_area == 4):
            map_size_x[0] = x_center
            map_size_y[1] = y_center
            map_size_z[0] = z_center
            area_list.append("seu")
        elif(current_area == 5):
            map_size_x[1] = x_center
            map_size_y[0] = y_center
            map_size_z[1] = z_center
            area_list.append("nwd")
        elif(current_area == 6):
            map_size_x[0] = x_center
            map_size_y[0] = y_center
            map_size_z[1] = z_center
            area_list.append("ned")
        elif(current_area == 7):
            map_size_x[1] = x_center
            map_size_y[1] = y_center
            map_size_z[1] = z_center
            area_list.append("swd")
        elif(current_area == 8):
            map_size_x[0] = x_center
            map_size_y[1] = y_center
            map_size_z[1] = z_center
            area_list.append("sed")
    return area_list

print(octreeList(map_size_x,map_size_y,map_size_z,coord_x,coord_y,coord_z,depth))
ax.scatter(coord_x,coord_y,coord_z,color="red")
plt.show()