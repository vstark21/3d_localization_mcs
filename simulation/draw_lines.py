from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

points1 = [[-23.126867297145786, -0.06765823467065786, 19.299539574047063], [4, 0, 4]]
points2 = [[-19.299539574047067, -0.0676582346706581, 23.126867297145782], [4, 0, 4]]

ax.plot([points1[0][0], points1[1][0]], [points1[0][1], points1[1][1]], [points1[0][2], points1[1][2]])
ax.plot([points2[0][0], points2[1][0]], [points2[0][1], points2[1][1]], [points2[0][2], points2[1][2]])

plt.show()
