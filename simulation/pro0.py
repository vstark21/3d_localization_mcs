import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")

huskypos = [2, 0, 0.1]
husky = p.loadURDF("bot.urdf", huskypos[0], huskypos[1], huskypos[2])

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(husky)
print(cubePos,cubeOrn)
p.disconnect()