from env import *
from refine import *

def rot_az(C, angle):
    C = np.array(C[:3] + [1])
    angle = math.radians(angle)
    R = [[math.cos(angle), math.sin(angle), 0, 0], 
        [-math.sin(angle), math.cos(angle), 0, 0],
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
    coords = [2 * (coords[0] / cam.width) - 1, 
              2 * (coords[1] / cam.height) - 1]

    Fc = math.sqrt(3) # 1/tan(FOV/2)

    # (Cax, Cay, Caz) are the cordinates of located centroid in the frame of a'th camera.
    Cay = 1
    Cax = (Cay * coords[0]) / 1
    Caz = (Cay * coords[1]) / 1

    # Rotation 
    if num == 1:
        # Cam - 1
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], -90)
    
    elif num == 2:
        # Cam - 2
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], 90)

    elif num == 3:
        # Cam - 3
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)
        Rx, Ry, Rz = rot_az([Rx, Ry, Rz], 180)

    elif num == 4:
        # Cam - 4
        Rx, Ry, Rz = rot_ax([Cax, Cay, Caz], -45)

    Rx = Rx - cam.pos[0]
    Ry = Ry - cam.pos[1]
    Rz = Rz - cam.pos[2]

    # print([Rx, Ry, Rz])
    return [Rx, Ry, Rz]


def process_images(cam1, cam2, cam3, cam4):
    global KILL_CAM_THREAD
    time.sleep(1)
    f = open("run.txt",'w')

    while not KILL_CAM_THREAD:
        stepSimulation()
        lines = []

        image = cam1.getCameraImage()
        cX, cY = get_center(image, 'CAM1_M')
        Rwc = main_process(cam1, [cX, cY], 1)
        f.write(f"{Rwc[0]} {Rwc[1]} {Rwc[2]}\n")
        lines.append(line3d(cam1.pos, Rwc))
        cv2.imshow('CAM1', image)

        image = cam2.getCameraImage()
        cX, cY = get_center(image, 'CAM2_M')
        Rwc = main_process(cam2, [cX, cY], 2)
        f.write(f"{Rwc[0]} {Rwc[1]} {Rwc[2]}\n")
        lines.append(line3d(cam2.pos, Rwc))
        cv2.imshow('CAM2', image)

        image = cam3.getCameraImage()
        cX, cY = get_center(image, 'CAM3_M')
        Rwc = main_process(cam3, [cX, cY], 3)
        f.write(f"{Rwc[0]} {Rwc[1]} {Rwc[2]}\n")
        lines.append(line3d(cam3.pos, Rwc))
        cv2.imshow('CAM3', image)

        image = cam4.getCameraImage()
        cX, cY = get_center(image, 'CAM4_M')
        Rwc = main_process(cam4, [cX, cY], 4)
        f.write(f"{Rwc[0]} {Rwc[1]} {Rwc[2]}\n")
        lines.append(line3d(cam4.pos, Rwc))
        cv2.imshow('CAM4', image)

        output = refine(lines)
        print("OUTPUT : ", output)
        f.write(f"{output[0]} {output[1]} {output[2]}\n")
        # for li in lines:
        #     print(li.perp_dist([1, 0, 0.5]))

        cv2.waitKey(1)
    f.close()

if __name__ == '__main__':
    
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-9.8)

    planeId = p.loadURDF("plane.urdf")

    cont = Controller([2, 0, 0])

    if USE_CAMS:
        temp = 4
        cam1 = Camera([temp, 0, temp], [-1, 0, 0])
        cam2 = Camera([-temp, 0, temp], [1, 0, 0])
        cam3 = Camera([0, temp, temp], [0, -1, 0])
        cam4 = Camera([0, -temp, temp], [0, 1, 0])
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
    # cont.move_in_sqaure()
    # cont.move([5, 0])
    try:
        for i in range(10000):
            stepSimulation()
    except KeyboardInterrupt:
        print("Manual Interruption Occured")
        
    # cont.move_in_circle()
    # cont.plot_position()
    if USE_CAMS:
        KILL_CAM_THREAD = True
        cam_thread.join()
    p.disconnect()
    cv2.destroyAllWindows()