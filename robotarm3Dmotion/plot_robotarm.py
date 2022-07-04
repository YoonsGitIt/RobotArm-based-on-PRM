import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

def plot_arm(point):

    for i in range(len(point)-1):
        ax.plot([point[i,0],point[i+1,0]],[point[i,1],point[i+1,1]],[point[i,2],point[i+1,2]],color = 'k',linewidth = '5')
