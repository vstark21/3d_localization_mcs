from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def strip(line, words=3):
    a = []
    for i in range(words):
        t = ""
        for i in range(len(line)):
            if line[i] == " ":
                line = line[i+1:]
                break
            t += line[i]
        a.append(float(t))
    return a


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(-2, 2)
temp = 4
poses = [
    [temp, 0, temp],
    [-temp, 0, temp],
    [0, temp, temp], 
    [0, -temp, temp]
    ]

a = []
final_pos = []
 
with open('run.txt', 'r') as f:
    for _ in range(4):
        line = strip(f.readline())
        a.append(line)
    final_pos = strip(f.readline())



lines = []
for el1, el2 in zip(poses, a):
    ax.plot([el1[0], el2[0]], [el1[1], el2[1]], [el1[2], el2[2]])
    print(el2)

# ax.scatter([0], [0], [0.5], label="Ground Truth")


ax.scatter(*[[el] for el in final_pos], label="Estimated")

plt.legend()
plt.show()
