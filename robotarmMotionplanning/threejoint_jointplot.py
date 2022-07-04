from ast import And
from unittest import result
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import octree_prm
import numpy as np
import numpy.random as npr
import pandas as pd
import time
import math
import collisionChecker as coch
import forwardKinematics as fk
import print2Darm as p2a

d2r = math.pi/180
r2d = 180/math.pi
    
start_time = time.time()

def distance3D(pose1,pose2):
    dist = math.sqrt((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2 + (pose1[2]-pose2[2])**2)
    return dist
       
#initiallize
startJoint = np.array([-150,30,-10])
goalJoint = np.array([30,-30,-10])

fig = plt.figure(figsize=(20, 10))
ax = fig.add_subplot(111, projection='3d')
ax.tick_params(labelsize=23)

plt.yticks(fontsize=23)
plt.xticks(fontsize=23)

#generate random point
rand_num = 500
armPose = np.zeros((1,3),int)
armPose[0] = startJoint
for i in range(rand_num):
    np.random.seed(i)
    joint = npr.randint(-180,180,(1,3))
    ax.scatter(joint[0,0],joint[0,1],joint[0,2],color = 'r' ,marker='<',s=35 )
    if (coch.checkJoint(joint[0])):
        armPose = np.append(armPose,joint,axis=0)
    else:
        continue
ax.scatter(joint[0,0],joint[0,1],joint[0,2],color = 'r' ,marker='<',s=35 , label='Not available point' )
armPose=np.append(armPose,[goalJoint],axis=0)
print(len(armPose))


#make trees
pare = np.zeros((1,3))
add_pare = np.zeros((1,3))
child = np.zeros((len(armPose),len(armPose)))
lambda1 = 0.1
lambda2 = 1-lambda1
k_val=50
k = lambda1*k_val*0.5 + lambda2*k_val
for i in range(len(armPose)):
    t_num = 0
    for j in range(len(armPose)):
        currentPoint = fk.forwardkinematic(armPose[i])
        nextPoint = fk.forwardkinematic(armPose[j])
        current_dist = lambda1*distance3D(armPose[i],armPose[j]) + lambda2*math.sqrt((currentPoint[3,0]-nextPoint[3,0])**2 + (currentPoint[3,1]-nextPoint[3,1])**2)
        if( i!=j and 0 < current_dist < k  and coch.lineChecker(currentPoint[2],nextPoint[2]) and coch.lineChecker(currentPoint[3],nextPoint[3])):
            add_pare[0,0]=i
            add_pare[0,1]=j
            add_pare[0,2]=current_dist
            pare = np.append(pare,add_pare,axis=0)
            child[i,t_num] = j
            t_num = t_num +1
            ax.scatter(armPose[i,0],armPose[i,1],armPose[i,2],color='blue' )
            ax.scatter(armPose[j,0],armPose[j,1],armPose[j,2],color='blue')
pare = np.delete(pare,(0),axis=0)
# print(pare)
ax.scatter(armPose[0,0],armPose[0,1],armPose[0,2],color='blue' , label='available point')

print(len(pare))


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

startPoint = fk.forwardkinematic(startJoint)
goalPoint = fk.forwardkinematic(goalJoint)
end_time = time.time()
print(end_time - start_time)

for i in range(len(short_path)-1):
    child_idx = int(short_path[i])
    parent_idx = int(short_path[i+1])
    child_point = fk.forwardkinematic(armPose[child_idx])
    parent_point = fk.forwardkinematic(armPose[parent_idx])
    ax.plot([armPose[child_idx,0],armPose[parent_idx,0]],[armPose[child_idx,1],armPose[parent_idx,1]],[armPose[child_idx,2],armPose[parent_idx,2]],color = 'y',linewidth='7')
plt.legend(loc = 1,fontsize = 20)

ax.scatter(startJoint[0],startJoint[1],startJoint[2],color = 'k', s = 40)
ax.scatter(goalJoint[0],goalJoint[1],goalJoint[2],color = 'k', s =20)
ax.text(startJoint[0]+2,startJoint[1]+2,startJoint[2]+2,'start',color = 'k', fontsize = 20)
ax.text(goalJoint[0]+2,goalJoint[1]+2,goalJoint[2]+2,'goal',color = 'k', fontsize = 20)

print(k,rand_num)
plt.show()