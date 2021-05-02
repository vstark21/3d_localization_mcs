from utils import *
import itertools

class Camera:

    def __init__(self, pos, upVec, distort=False):
        self.pos = pos
        self.width = 512
        self.height = 512
        self.fov = 60
        self.f = np.array([self.width / math.tan(math.radians(self.fov / 2)), 
                        self.height / math.tan(math.radians(self.fov / 2))])
        self.aspect = self.width / self.height
        self.near = 0.02
        self.far = 15
        self.view_matrix = p.computeViewMatrix(pos, [0, 0, 0], upVec)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, 
                                                            self.aspect, 
                                                            self.near, 
                                                            self.far)
        self.dist_coeffs = [-0.25349, 0.11868, -0.00028, 0.00005, 0.0000]

        self.image = None
        self.distort = distort
    
    def getCameraImage(self):
        images = p.getCameraImage(self.width,
                            self.height,
                            self.view_matrix,
                            self.projection_matrix,
                            shadow=True,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
        self.image = (np.reshape(images[2], (self.height, self.width, 4)) * 1. / 255.)[:, :, :3][:, :, ::-1]

        if self.distort:
            self.image = self.distort_image(self.image)
        return self.image

    def distort_image(self, image):
        """Distort an image based on the cameras distortion coefficients.
        Args:
            image:  The undistorted image.
        Returns:
            The distorted image.
        """

        # this function is based on the formulas from here:
        # https://stackoverflow.com/a/58131157/2095383
        # Computations are done on numpy arrays as much as possible for
        # performance reasons.

        distorted_image = np.zeros_like(image)

        k_1, k_2, p_1, p_2, k_3 = self.dist_coeffs

        f = self.f
        c = np.array([self.width / 2, self.height / 2])

        image_points = np.array(
            tuple(
                itertools.product(
                    range(self.width), range(self.height)
                )
            )
        )

        # normalize the image coordinates
        norm_points = (image_points - c) / f
        norm_points_square = norm_points ** 2
        norm_points_xy = norm_points.prod(axis=1)

        # determining the radial distortion
        r2 = np.sum(norm_points_square, axis=1)
        icdist = 1 / (1 - ((k_3 * r2 + k_2) * r2 + k_1) * r2)

        # determining the tangential distortion
        p = np.array([[p_2, p_1]])

        r2_plus_2_point_sq = r2[:, None] + 2 * norm_points_square
        delta = 2 * p * norm_points_xy[:, None] + p[::-1] * r2_plus_2_point_sq

        distorted_points = (norm_points + delta) * icdist[:, None]

        # un-normalise
        distorted_points = distorted_points * f + c

        # float to int
        distorted_points = distorted_points.round().astype(int)

        # filter out points that are outside the image
        in_image_idx = np.all(
            np.logical_and(
                (0, 0) <= distorted_points,
                distorted_points < (self.width, self.height),
            ),
            axis=1,
        )
        distorted_points = distorted_points[in_image_idx]
        image_points = image_points[in_image_idx]

        # finally construct the distorted image
        distorted_image[tuple(distorted_points.T)] = image[
            tuple(image_points.T)
        ]

        return distorted_image



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
