from utils import *

class Camera:

    def __init__(self, pos, upVec):
        self.pos = pos
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

    def distort_image(self):
        return 


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
        self.position_list = [[], [], []]
        self.changeState()

    def changeState(self):
        self.position, self.orientation = p.getBasePositionAndOrientation(self.husky)
        self.orientation = p.getEulerFromQuaternion(self.orientation)

        self.position = roundList(self.position, 5)

        self.position_list[0].append(self.position[0])
        self.position_list[1].append(self.position[1])
        self.position_list[2].append(self.position[2])
        # print(self.orientation, self.position)

    def stop(self, maxForce=100):
        p.setJointMotorControl(self.husky, self.front_left, p.VELOCITY_CONTROL, 0, maxForce)
        p.setJointMotorControl(self.husky, self.front_right, p.VELOCITY_CONTROL, 0, maxForce)
        p.setJointMotorControl(self.husky, self.rear_left, p.VELOCITY_CONTROL, 0, maxForce)
        p.setJointMotorControl(self.husky, self.rear_right, p.VELOCITY_CONTROL, 0, maxForce)

    def turn(self, point, velocity, maxForce, epsilon, K=5):
        vecPoint = [point[0] - self.position[0], point[1] - self.position[1]]
        err = getAngle(vecPoint, [1, 0]) - parse_angle(self.orientation[2])
        if err > math.pi:
            err = -(2*math.pi - err)
        if err < -math.pi:
            err = (2*math.pi + err)
        
        while abs(err) > epsilon:
            # print(err)
            err = K * (err/math.pi)
            # print(getAngle(vecPoint, [1, 0]), parse_angle(self.orientation[2]), vecPoint, err)

            p.setJointMotorControl(self.husky, self.front_left, p.VELOCITY_CONTROL, -err*velocity, maxForce)
            p.setJointMotorControl(self.husky, self.front_right, p.VELOCITY_CONTROL, err*velocity, maxForce)
            p.setJointMotorControl(self.husky, self.rear_left, p.VELOCITY_CONTROL, -err*velocity, maxForce)
            p.setJointMotorControl(self.husky, self.rear_right, p.VELOCITY_CONTROL, err*velocity, maxForce)
            self.changeState()  
            vecPoint = [point[0] - self.position[0], point[1] - self.position[1]]
            err = getAngle(vecPoint, [1, 0]) - parse_angle(self.orientation[2])
            if err > math.pi:
                err = -(2*math.pi - err)
            if err < -math.pi:
                err = (2*math.pi + err)
            
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
    
    def move_in_square(self, length=2):
        self.move([0, length])
        self.move([length, length])
        self.move([length, 0])
        self.move([0, 0])

    def move_in_circle(self, radius=2, segments=24):
        self.move([radius, 0])
        theta = 2 * math.pi / segments
        for i in range(1, segments+1):
            x = radius * math.cos(i*theta)
            y = radius * math.sin(i*theta)
            self.move([x, y])

    def plot_position(self, est_cords=[]):
        plt.plot(*self.position_list[:2])
        if est_cords:
            plt.plot(*est_cords)
        plt.show()
