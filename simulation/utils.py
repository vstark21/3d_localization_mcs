from env import *
from refine import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def get_world_coords(cam, coords, num=1):
    coords = [2 * (coords[0] / cam.width) - 1, 
              1 - 2 * (coords[1] / cam.height)]

    Fc = math.sqrt(3) # 1/tan(FOV/2)

    # (Cax, Cay, Caz) are the cordinates of located centroid in the frame of a'th camera.
    Cay = 3
    Cax = (Cay * coords[0]) / Fc
    Caz = (Cay * coords[1]) / Fc

    mat = create_transformation_matrix(cam.pos)

    Rx, Ry, Rz = np.matmul(mat, [Cax, Cay, Caz, 1])[:3]

    # print([Rx, Ry, Rz])
    return [Rx, Ry, Rz]

def plot_trajectory(grdts, ests):
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    maxn = float("-inf")
    maxn = max(maxn, np.max(grdts))
    maxn = max(maxn, np.max(ests))

    minn = float("inf")
    minn = min(minn, np.min(grdts))
    minn = min(minn, np.min(ests))

    ax.set_xlim3d(minn, maxn)
    ax.set_ylim3d(minn, maxn)
    ax.set_zlim3d(minn, maxn)

    ax.plot(*grdts, label="Ground Truth Traj")
    ax.plot(*ests, label="Estimated Traj")

    plt.legend()
    plt.show()

def normalize(a):
    mod = math.sqrt((a[0]*a[0]) + (a[1]*a[1]) + (a[2]*a[2]))
    a[0] /= mod
    a[1] /= mod
    a[2] /= mod

    return a

def create_transformation_matrix(pos):

    # Cameras are looking at [0, 0, 0]
    look = [0, 0, 0]
    mat = [ [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0] ]
    mat[0][3] = pos[0]
    mat[1][3] = pos[1]
    mat[2][3] = pos[2]
    mat[3][3] = 1

    upVec = normalize([-pos[0], -pos[1], (pos[0]*pos[0] + pos[1]*pos[1])/pos[2]])

    direc = normalize([look[0] - pos[0], look[1] - pos[1], look[2] - pos[2]])

    right = normalize(np.cross(direc, upVec).tolist())

    newUp = np.cross(right, direc).tolist()

    mat[0][0] = right[0]
    mat[1][0] = right[1]
    mat[2][0] = right[2]
    mat[3][0] = 0

    mat[0][1] = direc[0]
    mat[1][1] = direc[1]
    mat[2][1] = direc[2]
    mat[3][1] = 0
    
    mat[0][2] = newUp[0]
    mat[1][2] = newUp[1]
    mat[2][2] = newUp[2]
    mat[3][2] = 0
    
    return mat
