from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

points1 = [[ 0, 0,  0.5], [4, 0, 4]]
points2 = [[1.6873132702854212, -0.006765823467065786, 2.070046042595293], [4, 0, 4]]

ax.plot([points1[0][0], points1[1][0]], [points1[0][1], points1[1][1]], [points1[0][2], points1[1][2]])
ax.plot([points2[0][0], points2[1][0]], [points2[0][1], points2[1][1]], [points2[0][2], points2[1][2]])

plt.show()
