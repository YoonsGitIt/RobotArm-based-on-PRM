from tracemalloc import start
from unittest import result
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import numpy.random as npr
import pandas as pd
import time
import math
import colliChecker as checker
import forwardKinematics as fk
import distance as dist

d2r = math.pi/180
r2d = 180/math.pi
    
#circle
circle_center = np.array([[10,30,80],[50,70,40]])
circle_radius = np.array([20,20])
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

ax.set_xlim3d(0, 100)
ax.set_ylim3d(0, 100)
ax.set_zlim3d(0, 100)
ax.scatter(0,0,0,color='red')

for i in range(len(circle_radius)):
    r = circle_radius[i]
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = r * np.outer(np.cos(u), np.sin(v)) + circle_center[i,0]
    y = r * np.outer(np.sin(u), np.sin(v)) + circle_center[i,1]
    z = r * np.outer(np.ones(np.size(u)), np.cos(v)) + circle_center[i,2]
    ax.plot_surface(x, y, z, color='blue', alpha=0.7)

start_time = time.time()

#initiallize
startJoint = np.array([0,0,0,0,0,0])
goalJoint = np.array([30,-30,-90,90,60,0])

#generate random point
rand_num = 200
armPose = np.zeros((1,6),int)
armPose[0] = startJoint
for i in range(rand_num):
    np.random.seed(i)
    joint = npr.randint(-180,180,(1,6))
    if (checker.checkJoint(joint[0])):
        armPose = np.append(armPose,joint,axis=0)
    else:
        continue
armPose=np.append(armPose,[goalJoint],axis=0)
print(len(armPose))

#make trees
k=100
pare = np.zeros((1,6))
add_pare = np.zeros((1,6))
child = np.zeros((len(armPose),len(armPose)))
lambda1 = 0.6
lambda2 = 0.4
for i in range(len(armPose)):
    t_num = 0
    for j in range(len(armPose)):
        currentPoint = fk.forwardkinematic(armPose[i])
        nextPoint = fk.forwardkinematic(armPose[j])
        current_dist = dist.distance(armPose[i],armPose[j]) 
        if( 0 < current_dist < k  and checker.lineChecker(currentPoint[2],nextPoint[2]) and checker.lineChecker(currentPoint[3],nextPoint[3])):
        # if( and lineChecker(currentPoint[1],nextPoint[1]) and lineChecker(currentPoint[2],nextPoint[2]) and lineChecker(currentPoint[3],nextPoint[3])):
            add_pare[0,0]=i
            add_pare[0,1]=j
            add_pare[0,2]=current_dist
            pare = np.append(pare,add_pare,axis=0)
            child[i,t_num] = j
            t_num = t_num +1
pare = np.delete(pare,(0),axis=0)
# print(pare)
print(len(pare))

def plot_arm(point):
    
    for i in range(len(point)-1):
        ax.plot([point[i,0],point[i+1,0]],[point[i,1],point[i+1,1]],[point[i,2],point[i+1,2]],color = 'k',linewidth = '5')

##### difkstra_algorithm #####

#initiall
vertex_size = len(armPose)
Q = np.arange(vertex_size)
large_N = float('inf')
# S = np.array([])
S = np.empty(shape=0)
U_w = large_N * np.ones((len(armPose),len(armPose)))
iteration = 0
limit_iter = 1e4
u_idx_pre = 0

#the queue initially contains all vertices
for row_idx in range(len(pare)):
    temp = pare[row_idx,:]
    coord_vertex = np.asarray(temp[0:2],dtype=int)
    distance_vertex = temp[2]
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
print(len(short_path))

end_time = time.time()
print(end_time-start_time)

# plot_arm(fk.forwardkinematic(startJoint))
plot_arm(fk.forwardkinematic(goalJoint))
plt.show()