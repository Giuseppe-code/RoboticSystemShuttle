
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np

from lib.dds.dds import *
from lib.dds.image_reader import *
from lib.utils.time import *
from lib.system.controllers import *
from manipulator_control import *
from object_finder import *


displ_x_controller = PID_Controller(0.0005, 0, 0, 0.1)
displ_y_controller = PID_Controller(0.0005, 0, 0, 0.1)

robot = FourJointsManipulatorControl()

x_target_robot = 0.8
y_target_robot = 0.0
z_target_robot = 0.5
robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))

dds = DDS()
dds.start()

dds.subscribe(['tick'])

t = Time()
t.start()

imr = ImageReader('localhost', 4445)
imr.connect()
target_color = sys.argv[1] if len(sys.argv) > 1 else "red"
obj_finder = ObjectFinder(target_color)
while True:
    dds.wait('tick')

    delta_t = t.elapsed()

    ## image processing
    img = imr.request_image(dds, 512, 512)

    cx, cy, binary_image = obj_finder.find(img)

    if (cx >= 0)and(cy >= 0):
        errx = (256 - cx)
        erry = (256 - cy)

        displ_x = displ_x_controller.evaluate(delta_t, errx)
        displ_y = displ_y_controller.evaluate(delta_t, erry)

        print('Error vs. image center ', (errx, erry), ' - Displacement ', (displ_x, displ_y))

        if (abs(errx) < 5)and(abs(erry) < 5):
            z_target_robot -= 0.01
            if z_target_robot < 0.02:
                print("Object got")
                break

        (x, y, z, a) = robot.get_pose()
        x_target_robot = x + displ_x
        y_target_robot = y + displ_y

        print((x,y), " - " , (x_target_robot, y_target_robot))
        if not(robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))):
            x_target_robot -= displ_x
            y_target_robot -= displ_y


    # robot control
    robot.evaluate(delta_t)
    (t0, t1, t2, t3) = robot.get_joint_angles()
    (x, y, z, a) = robot.get_pose()

    dds.publish('theta0', t0, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta1', t1, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta2', t2, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta3', t3, DDS.DDS_TYPE_FLOAT)
    dds.publish('x', x, DDS.DDS_TYPE_FLOAT)
    dds.publish('y', y, DDS.DDS_TYPE_FLOAT)
    dds.publish('z', z, DDS.DDS_TYPE_FLOAT)
    dds.publish('a', a, DDS.DDS_TYPE_FLOAT)

    # image show
    cv2.imshow('image', img)
    cv2.imshow('binary_image', binary_image)

    k = cv2.waitKey(1)
    if k == ord('q'):
        break

dds.stop()
