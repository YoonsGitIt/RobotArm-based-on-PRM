import matplotlib.pyplot as plt

map_size_x = list(map(float,input("Please input x size >>")).split())
map_size_y = list(map(float,input("Please input y size >>")).split())
coord_x = float(input("Input x coordinate : "))
coord_y = float(input("Input y coordinate : "))
depth = int(input("Input depth : "))

def check_area (size_x,size_y,x,y):

    x_center = float(size_x[0] + size_x[1])/2
    y_center = float(size_y[0] + size_y[1])/2
    if (x < x_center and  y >= y_center):
        area = 1
    elif (x >= x_center and y >= y_center):
        area = 2
    elif (x < x_center and y < y_center):
        area = 3
    elif (x >= x_center and y < y_center):
        area = 4
    return area

plt.scatter(coord_x,coord_y,color="red")
area_list = []
for i in range(depth):
    
    x_center = (map_size_x[0] + map_size_x[1])/2
    y_center = (map_size_y[0] + map_size_y[1])/2
    plt.plot([map_size_x[0],map_size_x[1]],[y_center,y_center],color = "black")
    plt.plot([x_center,x_center],[map_size_y[0],map_size_y[1]],color = "black")

    current_area = check_area(map_size_x,map_size_y,coord_x,coord_y)
    if(current_area == 1):
        map_size_x[1] = x_center
        map_size_y[0] = y_center
        area_list.append("nw")
    elif(current_area == 2):
        map_size_x[0] = x_center
        map_size_y[0] = y_center
        area_list.append("ne")
    elif(current_area == 3):
        map_size_x[1] = x_center
        map_size_y[1] = y_center
        area_list.append("sw")
    elif(current_area == 4):
        map_size_x[0] = x_center
        map_size_y[1] = y_center
        area_list.append("se")

print(area_list)
plt.title("Plot quadtree")
plt.show()