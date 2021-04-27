from env import *
from refine import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def rot_az(C, angle):
    C = np.array(C[:3] + [1])
    angle = math.radians(angle)
    R = [[math.cos(angle), -math.sin(angle), 0, 0], 
        [math.sin(angle), math.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
    R = np.array(R)
    return list(np.matmul(R, C))[:3]

def rot_ax(C, angle):
    C = np.array(C[:3] + [1])
    angle = math.radians(angle)
    R = [[1, 0, 0, 0], 
        [0, math.cos(angle), -math.sin(angle), 0], 
        [0, math.sin(angle), math.cos(angle), 0], 
        [0, 0, 0, 1]]
    R = np.array(R)
    return list(np.matmul(R, C))[:3]

def rot_ay(C, angle):
    C = np.array(C[:3] + [1])
    angle = math.radians(angle)
    R = [[math.cos(angle), 0, math.sin(angle), 0],
        [0, 1, 0, 0],
        [-math.sin(angle), 0, math.cos(angle), 0],
        [0, 0, 0, 1]]
    R = np.array(R)
    return list(np.matmul(R, C))[:3]


def get_world_coords(cam, coords, num=1):
    coords = [2 * (coords[0] / cam.width) - 1, 
              1 - 2 * (coords[1] / cam.height)]

    Fc = math.sqrt(3) # 1/tan(FOV/2)

    # (Cax, Cay, Caz) are the cordinates of located centroid in the frame of a'th camera.
    Cay = 3
    Cax = (Cay * coords[0]) / Fc
    Caz = (Cay * coords[1]) / Fc

    # Rotation 
    if num == 1:
        # Cam - 1
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], 90)
    
    elif num == 2:
        # Cam - 2
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], -90)

    elif num == 3:
        # Cam - 3
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], 180)

    elif num == 4:
        # Cam - 4
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)

    # print([Rx, Ry, Rz])
    Rx = Rx + cam.pos[0]
    Ry = Ry + cam.pos[1]
    Rz = Rz + cam.pos[2]

    # print([Rx, Ry, Rz])
    return [Rx, Ry, Rz]

def plot_trajectory(grdts, ests):
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(*grdts, label="Ground Truth Traj")
    ax.plot(*ests, label="Estimated Traj")

    plt.legend()
    plt.show()
