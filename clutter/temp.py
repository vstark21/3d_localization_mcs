import numpy as np
import math

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

def main_process(cam, coords, num=1):
    coords = [2 * (coords[0] / 512) - 1, 
              (2 * (coords[1] / 512) - 1)*-1]
    print(coords)

    Fc = math.sqrt(3) # 1/tan(FOV/2)

    # (Cax, Cay, Caz) are the cordinates of located centroid in the frame of a'th camera.
    Cay = Fc
    Cax = (Cay * coords[0]) / Fc
    Caz = (Cay * coords[1]) / Fc

    # Rotation 
    if num == 1:
        # Cam - 1
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], 45)
        print(Rx, Ry, Rz)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], 90)
    
    elif num == 2:
        # Cam - 2
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], 45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], -90)

    elif num == 3:
        # Cam - 3
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], 45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], 180)

    elif num == 4:
        # Cam - 4
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], 45)

    Rx = Rx + 4
    Ry = Ry + 0
    Rz = -Rz + 4

    return [Rx, Ry, Rz]


C = [255, 216]
print(main_process(1, C, 1))

C = [1, 0, 1]
print(rot_az(C, 90))
