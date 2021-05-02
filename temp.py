import math
import itertools
import numpy as np
import cv2

def distort_image(image):
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

        k_1, k_2, p_1, p_2, k_3 = [-0.25349, 0.11868, -0.00028, 0.00005, 0.0000]

        f = np.array([512*math.sqrt(3), 512*math.sqrt(3)])
        c = np.array([256, 256])

        image_points = np.array(
            tuple(
                itertools.product(
                    range(512), range(512)
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
                distorted_points < (512, 512),
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

def undistort_point(point, dist_coeffs, f=[math.sqrt(3), math.sqrt(3)]):
    point = np.array(point)
    f = np.array(f) * 512
    c = np.array([256, 256])
    point = (point - c) / f
    r = (point[0] * point[0] + point[1] * point[1])

    dp = np.array([
        2 * dist_coeffs[2] * point[0] * point[1] + dist_coeffs[3] * (r + 2 * (point[0] * point[0])),
        dist_coeffs[2] * (r + 2 * (point[1] * point[1])) +  2 * dist_coeffs[3] * point[0] * point[1]])
    pd = (1 + dist_coeffs[0] * r + dist_coeffs[1] * r * r + dist_coeffs[4] * r * r * r) * point + dp

    xp, yp = pd * f + c

    return [xp, yp]
    

def get_center(image, win_name="CAM"):
    image = np.array(image*255, dtype=np.uint8)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    mask = mask1 + mask2
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours.sort(key=lambda x:cv2.contourArea(x), reverse=True)

    M = cv2.moments(contours[0])
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    if False:
        cv2.imshow(win_name, mask)

    return cX, cY


image = cv2.imread("image1.png")
dist = distort_image(image)

cv2.imshow("ORIG", image)
print("1 : ", get_center(image))
cv2.imshow("DIST", dist)
print("2 : ", get_center(dist))
print(undistort_point(get_center(dist), [-0.25349, 0.11868, -0.00028, 0.00005, 0.0000]))
cv2.waitKey(0)
