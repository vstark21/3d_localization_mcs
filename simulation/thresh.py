import cv2
import numpy as np

def get_center(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(hsv,lower_red,upper_red)

    mask = mask1 + mask2
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours.sort(key=lambda x:cv2.contourArea(x), reverse=True)

    M = cv2.moments(contours[0])
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return cX, cY

M = np.array([1.7320507764816284, 0.0, 0.0, 0.0, 0.0, 1.7320507764816284, 0.0, 0.0, 0.0, 0.0, -1.0026702880859375, -1.0, 0.0, 0.0, -0.04005340486764908, 0.0])
M = M.reshape((4, 4))
M = np.array(list(M[:2]) + list([M[-1]]))
X = np.array([0.2, 0.2, 0, 1]).T
print(X)
x, y, _ = np.matmul(M, X)
print(M.shape)

Aw = [[M[1-1][1-1] - M[3-1][1-1]*x, M[1-1][2-1] - M[3-1][2-1]*x, M[1-1][3-1] - M[3-1][3-1]*x],
    [M[2-1][1-1] - M[3-1][1-1]*y, M[2-1][2-1] - M[3-1][2-1]*y, M[2-1][3-1] - M[3-1][3-1]*y]]

Aw = np.array(Aw)

Bw = np.array([x - M[1-1][4-1], y - M[2-1][4-1]]).T
P = np.linalg.inv(np.matmul(Aw.T, Aw))
P = np.matmul(np.matmul(P, Aw.T), Bw)
print(np.round(P, 3))