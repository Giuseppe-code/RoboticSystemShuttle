#
# find_object_main.py
#

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np

# import os
# from pathlib import Path

# import PyQt5
# from PyQt5.QtWidgets import QWidget # others imports

# os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.fspath(
#     Path(PyQt5.__file__).resolve().parent / "Qt5" / "plugins"
# )

from lib.dds.dds import *
from lib.dds.image_reader import *
from lib.system.controllers import *
from lib.utils.time import *
from lib.data.dataplot import *

from object_finder import *
from manipulator_control import *

plt = DataPlotter()
plt.set_x("time")
plt.add_y("current", "Current")
plt.add_y("target", "Target")


robot = FourJointsManipulatorControl()

x_target_robot = 0.8
y_target_robot = 0.0
z_target_robot = 0.5
robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))

dds = DDS()
dds.start()

dds.subscribe(['tick'])

imr = ImageReader('localhost', 4445)
imr.connect()

target_color = sys.argv[1] if len(sys.argv) > 1 else "blue"
obj_finder = ObjectFinder(target_color)

x_tracker = PID_Controller(0.0001, 0.0, 0.0, 0.01)
y_tracker = PID_Controller(0.0001, 0.0, 0.0, 0.01)

track_on = False

target_locked_count = 0
target_locked = False

t = Time()
t.start()
while True:
    dds.wait('tick')
    delta_t = t.elapsed()

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

    ## plot
    plt.append_x(t.get())
    plt.append_y("target", robot.wref_2)
    plt.append_y("current", robot.arm.element_2.w)

    ## image processing
    img = imr.request_image(dds, 512, 512)
    cx, cy, binary_image = obj_finder.find(img)

    if (cx >= 0)and(cy >= 0):
        error_x = 256 - cx
        error_y = 256 - cy
        if track_on:
            x_displ = x_tracker.evaluate(delta_t, error_x)
            x_target_robot += x_displ
            y_displ = y_tracker.evaluate(delta_t, error_y)
            y_target_robot -= y_displ
            print(error_x, error_y, x_displ, y_displ)
            if (abs(error_x) <= 2)and(abs(error_y) <= 2):
                target_locked_count += 1
                if target_locked_count > 10:
                    target_locked = True
            else:
                target_locked_count = 0

            if target_locked:
                z_target_robot -= 0.1 * delta_t
                if z_target_robot <= 0.0:
                    print("Object got")
                    sys.exit(1)
            robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))

    # image show
    cv2.imshow('image', img)
    cv2.imshow('binary_image', binary_image)

    k = cv2.waitKeyEx(1)
    if (k & 0xff) == ord('q'):
        break
    elif (k & 0xff) == ord('t'):
        track_on = True
    elif k in (0xf703, 0xff53, 2555904): # right
        x_target_robot += 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))
    elif k in (0xf702, 0xff51, 2424832): # left
        x_target_robot -= 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))
    elif k in (0xf701, 0xff54, 2621440): # down
        y_target_robot -= 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))
    elif k in (0xf700, 0xff52, 2490368): # up
        y_target_robot += 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))
dds.stop()
