from turtle import circle
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

circle_center = np.array([[10,30,80],[50,70,40]])
circle_radius = np.array([20,20])
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(0, 500)
ax.set_ylim3d(0, 500)
ax.set_zlim3d(0, 500)
ax.scatter(0,0,0,color='red')

for i in range(len(circle_radius)):
    r = circle_radius[i]
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = r * np.outer(np.cos(u), np.sin(v)) + circle_center[i,0]
    y = r * np.outer(np.sin(u), np.sin(v)) + circle_center[i,1]
    z = r * np.outer(np.ones(np.size(u)), np.cos(v)) + circle_center[i,2]
    ax.plot_surface(x, y, z, color='blue', alpha=0.7)

plt.show()