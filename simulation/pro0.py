import pybullet as p
import time
import pybullet_data
import cv2
import numpy as np


class Camera:
    def __init__(self):
        self.width = 512
        self.height = 512
        self.fov = 60
        self.aspect = self.width / self.height
        self.near = 0.02
        self.far = 5
        self.view_matrix = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [1, 0, 0])
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, 
                                                            self.aspect, 
                                                            self.near, 
                                                            self.far)
    
    def getCameraImage(self):
        images = p.getCameraImage(self.width,
                            self.height,
                            self.view_matrix,
                            self.projection_matrix,
                            shadow=True,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
        return (np.reshape(images[2], (self.height, self.width, 4)) * 1. / 255.)



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")

huskypos = [0, 0, 0]
husky = p.loadURDF("bot.urdf", huskypos[0], huskypos[1], huskypos[2])

cam1 = Camera()

for i in range (10000):
    p.stepSimulation()
    cv2.imshow('CAM1', cam1.getCameraImage())
    cv2.waitKey(1)
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(husky)
print(cubePos,cubeOrn)
p.disconnect()
cv2.destroyAllWindows()