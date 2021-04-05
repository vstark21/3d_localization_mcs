import pybullet as p
import time
import pybullet_data
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import threading

def roundList(arr, dec=1):
    ans = []
    for el in arr:
        ans.append(round(el, dec))
    return ans

class Camera:

    def __init__(self, pos, upVec):
        self.width = 512
        self.height = 512
        self.fov = 60
        self.aspect = self.width / self.height
        self.near = 0.02
        self.far = 15
        self.view_matrix = p.computeViewMatrix(pos, [0, 0, 0], upVec)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, 
                                                            self.aspect, 
                                                            self.near, 
                                                            self.far)

        self.image = None
    
    def getCameraImage(self):
        images = p.getCameraImage(self.width,
                            self.height,
                            self.view_matrix,
                            self.projection_matrix,
                            shadow=True,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
        self.image = (np.reshape(images[2], (self.height, self.width, 4)) * 1. / 255.)[:, :, :3][:, :, ::-1]
        return self.image

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

def stepSimulation():
    p.stepSimulation()
    time.sleep(1./240.)


class Controller:

    def __init__(self, init_pos=[0, 0, 0]):

        self.husky = p.loadURDF("bot.urdf", *init_pos)
        self.front_left = 2
        self.front_right = 3
        self.rear_left = 4
        self.rear_right = 5
        self.position = None
        self.orientation = None
        self.position_list = [[], []]
        self.changeState()

    def changeState(self):
        self.position, self.orientation = p.getBasePositionAndOrientation(self.husky)
        self.orientation = p.getEulerFromQuaternion(self.orientation)

        self.position = roundList(self.position, 5)

        self.position_list[0].append(self.position[0])
        self.position_list[1].append(self.position[1])
        # print(self.orientation, self.position)

    def stop(self, maxForce=100):
        p.setJointMotorControl(self.husky, self.front_left, p.VELOCITY_CONTROL, 0, maxForce)
        p.setJointMotorControl(self.husky, self.front_right, p.VELOCITY_CONTROL, 0, maxForce)
        p.setJointMotorControl(self.husky, self.rear_left, p.VELOCITY_CONTROL, 0, maxForce)
        p.setJointMotorControl(self.husky, self.rear_right, p.VELOCITY_CONTROL, 0, maxForce)

    def turn(self, point, velocity, maxForce, epsilon, K=5):
        vecPoint = [point[0] - self.position[0], point[1] - self.position[1]]
        err = getAngle(vecPoint, [1, 0]) - parse_angle(self.orientation[2])
        
        while abs(err) > epsilon:
            # print(err)
            # print(getAngle(vecPoint, [1, 0]), parse_angle(self.orientation[2]), vecPoint)
            if abs(err) > math.pi:
                err = -(2*math.pi - err)
            if abs(err) < -math.pi:
                err = (2*math.pi + err)

            err = K * (err/math.pi)
            p.setJointMotorControl(self.husky, self.front_left, p.VELOCITY_CONTROL, -err*velocity, maxForce)
            p.setJointMotorControl(self.husky, self.front_right, p.VELOCITY_CONTROL, err*velocity, maxForce)
            p.setJointMotorControl(self.husky, self.rear_left, p.VELOCITY_CONTROL, -err*velocity, maxForce)
            p.setJointMotorControl(self.husky, self.rear_right, p.VELOCITY_CONTROL, err*velocity, maxForce)
            self.changeState()  
            vecPoint = [point[0] - self.position[0], point[1] - self.position[1]]
            err = getAngle(vecPoint, [1, 0]) - parse_angle(self.orientation[2])
            
            stepSimulation()
        self.stop()

    def move(self, point, velocity=5, maxForce=100, epsilon=0.25):
        assert len(point) == 2, "len(point) must be equal to 2"

        while math.sqrt((point[0]-self.position[0])**2 + (point[1]-self.position[1])**2) > epsilon:
            self.turn(point, velocity*2, maxForce, epsilon)
            p.setJointMotorControl(self.husky, self.front_left, p.VELOCITY_CONTROL, velocity, maxForce)
            p.setJointMotorControl(self.husky, self.front_right, p.VELOCITY_CONTROL, velocity, maxForce)
            p.setJointMotorControl(self.husky, self.rear_left, p.VELOCITY_CONTROL, velocity, maxForce)
            p.setJointMotorControl(self.husky, self.rear_right, p.VELOCITY_CONTROL, velocity, maxForce)
            self.changeState()
            stepSimulation()   
            # print("MOVING FORWARD")
        self.stop()

        self.changeState()  
    
    def move_in_sqaure(self, length=2):
        self.move([0, length])
        self.move([length, length])
        self.move([length, 0])
        self.move([0, 0])

    def plot_position(self):
        plt.plot(*self.position_list)
        plt.show()
    
def process_images(cam1, cam2, cam3, cam4):
    global KILL_CAM_THREAD
    while not KILL_CAM_THREAD:
        stepSimulation()
        cv2.imshow('CAM1', cam1.getCameraImage())
        cv2.imshow('CAM2', cam2.getCameraImage())
        cv2.imshow('CAM3', cam3.getCameraImage())
        cv2.imshow('CAM4', cam4.getCameraImage())

        cv2.waitKey(1)


global KILL_CAM_THREAD
KILL_CAM_THREAD = False
USE_CAMS = not False

if __name__ == '__main__':
    
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)

    planeId = p.loadURDF("plane.urdf")

    cont = Controller()

    if USE_CAMS:
        cam1 = Camera([5, 0, 2], [-1, 0, 0])
        cam2 = Camera([-5, 0, 2], [1, 0, 0])
        cam3 = Camera([0, 5, 2], [0, -1, 0])
        cam4 = Camera([0, -5, 2], [0, 1, 0])
        cam_thread = threading.Thread(target=process_images, args=(cam1, cam2, cam3, cam4))
        cam_thread.start()

    # k = 0.1
    # for i in range (10000):
    #     stepSimulation()
        
    #     cont.move([0, k])
    #     k+=0.1
    #     print(k)
        # cont.move([2, 2])
        # cont.move([2, 0])
        # cont.move([0, 0])


        # cubePos, cubeOrn = p.getBasePositionAndOrientation(husky)
        # cubePos = roundList(cubePos)
        # print(cubePos)
    cont.move_in_sqaure()
    # cont.move([0,5])
    cont.plot_position()
    if USE_CAMS:
        KILL_CAM_THREAD = True
        cam_thread.join()
    p.disconnect()
    cv2.destroyAllWindows()