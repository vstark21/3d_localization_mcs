from utils import *
from env import *
from refine import *

global EST_CORDS, KILL_CAM_THREAD
KILL_CAM_THREAD = False
USE_CAMS, USE_DISTORTION = True, False
EST_CORDS = [[], [], []]


def process_images(cam1, cam2, cam3, cam4):
    global KILL_CAM_THREAD, EST_CORDS
    time.sleep(1)

    while not KILL_CAM_THREAD:
        stepSimulation()
        lines = []

        image = cam1.getCameraImage()
        cX, cY = get_center(image, 'CAM1_M')
        Rwc = get_world_coords(cam1, [cX, cY], 1)
        lines.append(line3d(cam1.pos, Rwc))
        cv2.imshow('CAM1', image)

        image = cam2.getCameraImage()
        cX, cY = get_center(image, 'CAM2_M')
        Rwc = get_world_coords(cam2, [cX, cY], 2)
        lines.append(line3d(cam2.pos, Rwc))
        cv2.imshow('CAM2', image)

        image = cam3.getCameraImage()
        cX, cY = get_center(image, 'CAM3_M')
        Rwc = get_world_coords(cam3, [cX, cY], 3)
        lines.append(line3d(cam3.pos, Rwc))
        cv2.imshow('CAM3', image)

        image = cam4.getCameraImage()
        cX, cY = get_center(image, 'CAM4_M')
        Rwc = get_world_coords(cam4, [cX, cY], 4)
        lines.append(line3d(cam4.pos, Rwc))
        cv2.imshow('CAM4', image)
        cv2.imwrite("image.png", image*255)

        output = refine(lines)
        print("OUTPUT : ", output)
        EST_CORDS[0].append(output[0])
        EST_CORDS[1].append(output[1])
        EST_CORDS[2].append(output[2] - 0.5-0.1)
       
        cv2.waitKey(1)

if __name__ == '__main__':
    
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-9.8)

    planeId = p.loadURDF("plane.urdf")

    cont = Controller([0, 0, 0])

    if USE_CAMS:
        temp = 4
        cam1 = Camera([temp, 0, temp], [0, 0, 1], USE_DISTORTION)
        cam2 = Camera([-temp, 0, temp], [0, 0, 1], USE_DISTORTION)
        cam3 = Camera([0, temp, temp], [0, 0, 1], USE_DISTORTION)
        cam4 = Camera([0, -temp, temp], [0, 0, 1], USE_DISTORTION)
        cam_thread = threading.Thread(target=process_images, args=(cam1, cam2, cam3, cam4))
        cam_thread.start()

    time.sleep(2)
    # cont.move_in_square()
    # cont.move([3, 3])
    # cont.move_in_circle()
    # if USE_CAMS:
    #     KILL_CAM_THREAD = True
    #     cam_thread.join()
    # plot_trajectory(cont.position_list, EST_CORDS)

    try:
        for i in range(100000):
            stepSimulation()
    except KeyboardInterrupt:
        print("Manual Interruption Occured")

    if USE_CAMS:
        KILL_CAM_THREAD = True
        cam_thread.join()
    
    p.disconnect()
    cv2.destroyAllWindows()