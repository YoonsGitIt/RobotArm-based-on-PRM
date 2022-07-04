import imp
from random import randint
from tabnanny import check
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as npr
import math
import time
import dist

#draw obstacle
circle_center = np.array([[20,30],[35,80],[50,50],[80,20],[80,70]])
circle_radius = np.array([15,20,10,15,15])
a = plt.axes(xlim=(0, 110), ylim=(0, 110))

for i in range(len(circle_radius)):
    c = plt.Circle(circle_center[i], circle_radius[i], fc='k', ec ='k')
    a.add_patch(c)
    a.set_aspect('equal') # X, Y축 1:1 비율로 설정

start_time = time.time()
#avoid obstacle
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

#initialize
start = np.array([10,100])
goal = np.array([100,20])

#generate random point
rand_num = 200
rand_point = np.zeros((1,2),int)
rand_point[0] = start
for i in range(rand_num):
    point = npr.randint(0,110,(1,2))
    if(avoid_obstacle(point)):
        rand_point = np.append(rand_point,point,axis=0)
    else:
        continue
rand_point = np.append(rand_point,[goal],axis=0)

#make trees
k=20
pare = np.zeros((1,3))
add_pare = np.zeros((1,3))
child = np.zeros((len(rand_point),len(rand_point)))
for i in range(len(rand_point)):
    t_num = 0
    for j in range(len(rand_point)):
        current_dist = dist.distance(rand_point[i],rand_point[j])
        #print(i,j,current_dist)
        if( 0 < current_dist < k and line_checker(rand_point[i],rand_point[j])):
            add_pare[0,0]=i
            add_pare[0,1]=j
            add_pare[0,2]=current_dist
            pare = np.append(pare,add_pare,axis=0)
            child[i,t_num] = j
            t_num = t_num +1

pare = np.delete(pare,(0),axis=0)
print(pare)

#draw the lines
for i in range(len(pare)):
    idx_0 = int(pare[i,0])
    idx_1 = int(pare[i,1])
    if(idx_0 < idx_1):
        plt.plot([rand_point[idx_0,0],rand_point[idx_1,0]],[rand_point[idx_0,1],rand_point[idx_1,1]],color = 'c',linewidth='0.5')
    else:
        continue


##### difkstra_algorithm #####

#initiall
vertex_size = len(rand_point)
Q = np.arange(vertex_size)
large_N = float('inf')
# S = np.array([])
S = np.empty(shape=0)
U_w = large_N * np.ones((len(rand_point),len(rand_point)))
iteration = 0
limit_iter = 1e4
u_idx_pre = 0

#the queue initially contains all vertices
for row_idx in range(len(pare)):
    temp = pare[row_idx,:]
    coord_vertex = np.asarray(temp[0:2],dtype=int)
    distance_vertex = temp[2]
    # print(coord_vertex[0],coord_vertex[1])
    U_w[coord_vertex[0],coord_vertex[1]] = distance_vertex
    U_w[coord_vertex[1],coord_vertex[0]] = distance_vertex
U_w[0,0] = 0
U_d = large_N * np.ones(len(U_w))
U_d[0] =0 

U_idx = 0
parent_tree = np.zeros((vertex_size,1))

#select the element of Q with min distance
while(len(Q) != 0):
    if (iteration > limit_iter):
        break
    min_d = min(U_d[Q])
    u_idx = np.where(U_d == min_d)
    u_idx_box = u_idx
    u_idx = u_idx[0]
    u_idx = u_idx_box[0][npr.randint(0,len(u_idx_box[0]))]
    u_idx_pre = u_idx
    S = np.append(S,[u_idx],axis=0)
    Q = np.setdiff1d(Q,S)

    u_idx_child = child[u_idx,:]
    u_non_zero = np.where(u_idx_child > 0)
    u_real_child = u_idx_child[u_non_zero]
    u_child_num = len(u_real_child)
    for idx in range(u_child_num):
        v_idx = int(u_real_child[idx])
        if U_d[v_idx] > (U_d[u_idx] + U_w[u_idx, v_idx]):
            U_d[v_idx] = U_d[u_idx] + U_w[u_idx, v_idx]
            parent_tree[v_idx] = u_idx
    iteration = iteration + 1

#draw shorttest line
find_idx = int(vertex_size-1)
find_tree = np.empty(shape=0)
find_tree = np.append(find_tree,[find_idx],axis=0)
while find_idx != 0:
    find_idx = int(parent_tree[find_idx,0])
    find_tree = np.append(find_tree,[find_idx],axis=0)
short_path = np.flip(find_tree)
for i in range(len(short_path)-1):
    child_idx = int(short_path[i])
    parent_idx = int(short_path[i+1])
    plt.plot([rand_point[child_idx,0],rand_point[parent_idx,0]],[rand_point[child_idx,1],rand_point[parent_idx,1]],color = 'r',linewidth='3')

end_time = time.time()
print(end_time - start_time)
plt.scatter(rand_point[:,0],rand_point[:,1],s=15,color = 'black',marker='.')
plt.text(start[0]+2,start[1]+2,'start')
plt.text(goal[0]+2,goal[1]+2,'goal')
plt.scatter(start[0],start[1],s=40,zorder=5,color = 'blue',marker='o')
plt.scatter(goal[0],goal[1],s=40,zorder=5,color = 'red',marker='o')
plt.show()