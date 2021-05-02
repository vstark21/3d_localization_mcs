import pybullet as p
import time
import pybullet_data
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import threading
from mpl_toolkits.mplot3d import Axes3D


def roundList(arr, dec=1):
    ans = []
    for el in arr:
        ans.append(round(el, dec))
    return ans

def norm(a):
    return math.sqrt(a[0]**2 + a[1]**2)

def getAngle(a, b):
    assert len(a) == 2 and len(b) == 2, "Length of a and b must be 2"
    angle = math.acos((a[0] * b[0] + a[1] * b[1])/(norm(a)*norm(b)))
    if a[0]*b[1] > a[1]*b[0]:
        angle = 2*math.pi - angle
    return angle

def parse_angle(angle):
    if angle < 0:
        angle = 2*math.pi + angle
    return angle

def get_center(image, win_name="CAM"):
    image = np.array(image*255, dtype=np.uint8)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(hsv,lower_red,upper_red)

    mask = mask1 + mask2
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    max_cnt = max(contours, key=lambda x:cv2.contourArea(x))

    M = cv2.moments(max_cnt)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    if False:
        cv2.imshow(win_name, mask)

    return cX, cY


def undistort_point(point, dist_coeffs, f, width, height):
    point = np.array(point)
    c = np.array([width / 2, height / 2])
    point = (point - c) / f
    r = (point[0] * point[0] + point[1] * point[1])

    dp = np.array([
        2 * dist_coeffs[2] * point[0] * point[1] + dist_coeffs[3] * (r + 2 * (point[0] * point[0])),
        dist_coeffs[2] * (r + 2 * (point[1] * point[1])) +  2 * dist_coeffs[3] * point[0] * point[1]])
    pd = (1 + dist_coeffs[0] * r + dist_coeffs[1] * r * r + dist_coeffs[4] * r * r * r) * point + dp

    xp, yp = pd * f + c

    return [xp, yp]


def get_world_coords(cam, coords, num=1):

    if cam.distort:
        coords = undistort_point(coords, cam.dist_coeffs, cam.f, cam.width, cam.height)

    coords = [2 * (coords[0] / cam.width) - 1, 
              1 - 2 * (coords[1] / cam.height)]

    Fc = [1 / math.tan(math.radians(cam.fov / 2)),
            1 / math.tan(math.radians(cam.fov / 2))] # 1/tan(FOV/2)

    # (Cax, Cay, Caz) are the cordinates of located centroid in the frame of a'th camera.
    Cay = 3
    Cax = (Cay * coords[0]) / Fc[0]
    Caz = (Cay * coords[1]) / Fc[1]

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

class line3d:
    def __init__(self, a, b):
        """
        a : a point on line
        b : a point on line
        """
        self.x1, self.y1, self.z1 = b
        self.l, self.m, self.n = a[0]-b[0], a[1]-b[1], a[2]-b[2]
    
    def perp_dist(self, p):
        """
        p : list of 3 integers
        """
        a, b, c = self.x1 - p[0], self.y1 - p[1], self.z1 - p[2]
        l, m, n = self.l, self.m, self.n
        
        temp = list(np.cross([a,b,c],[l,m,n]))

        temp = math.sqrt((temp[0] * temp[0]) + (temp[1] * temp[1]) + (temp[2] * temp[2]))
        temp = temp / (math.sqrt((l*l)+(m*m)+(n*n)))
        return abs(temp)
        