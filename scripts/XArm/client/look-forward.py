#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
"""
import sys
import math
import time
import datetime
import random
import traceback
import threading

# IKFast for xarm 7
import pyikfast
import numpy as np

"""
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
# git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
# cd xArm-Python-SDK
# python setup.py install
"""
try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

frontForwardAngle = [0, 2.5, 0, 37.3, 0, -57.3, 0]
frontBackAngle = [0.0,-45.0,0.0,0.0,0.0,-45.0,0.0]


pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

arm = XArmAPI('192.168.4.15')
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

variables = {}
params = {'speed': 50, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}

params['angle_acc'] = 50
params['angle_speed'] = 1000

# Register error/warn changed callback
def error_warn_change_callback(data):
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
            params['quit'] = True
            pprint('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)
arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        if not params['quit']:
            pprint('counter val: {}'.format(data['count']))
    arm.register_count_changed_callback(count_changed_callback)


# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)
arm.register_connect_changed_callback(connect_changed_callback)

# Rotation
if not params['quit']:
    # params['angle_acc'] = 1145
    # params['angle_speed'] = 80
    # if params['quit']:
    
    if arm.error_code == 0 and not params['quit']:
        # code = arm.set_servo_angle(angle=[0.1, -34.9, -0.1, 1.6, 0, -63.5, 0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        code = arm.set_servo_angle(angle=frontForwardAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))


# look forward but retracted
code = arm.set_servo_angle(angle=frontBackAngle, peed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

# print(arm.get_position(), arm.get_position(is_radian=True))

# angles = list(np.radians(frontForwardAngle))
# angles = list(np.radians(frontBackAngle))
# print("start (joints): ", angles)
# translate, rotate  = pyikfast.forward(angles)
# print("start pos (translate, rotate): ", translate, rotate, "\n")

# translate = [0.400, 0.0, 0.400]

# results = pyikfast.inverse(translate, rotate)

# for result in results: 
#     theseangles = list(result)
#     print("final angles (IK joints): ", theseangles)

# finalangles = list(np.degrees(results[3]))

# arm.set_servo_angle(angle=finalangles, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

# translate, rotate  = pyikfast.forward(angles)
# print("final FK (translate, rotate): ", translate, rotate, "\n")


# frontBackAngle = [0.0,-45.0,0.0,0.0,0.0,-45.0,0.0]
# angles = np.radians(frontBackAngle)

# print("start (joints): ", angles)

# translate, rotate  = pyikfast.forward(list(angles))

# print("FK (translate, rotate): ", translate, rotate, "\n")

# joints = pyikfast.inverse(translate, rotate)





# look down
# code = arm.set_servo_angle(angle=[0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)


# relative moves

# arm.set_position(pitch=-88.0, relative=False, wait=True)


# arm.set_position(pitch=-10.0, relative=True, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(pitch=-10.0, relative=True, wait=True)

# testing face tracking
# arm.set_tool_position(pitch=10.0, wait=True)
# arm.set_tool_position(pitch=-20.0, wait=True)
# arm.set_tool_position(pitch=10.0, wait=True)

# arm.set_servo_angle(servo_id=1, angle=3, relative=True, is_radian=False, wait=True)
# arm.set_servo_angle(servo_id=1, angle=-6, relative=True, is_radian=False, wait=True)
# arm.set_servo_angle(servo_id=1, angle=3, relative=True, is_radian=False, wait=True)

# arm.set_position(pitch=-20.0, relative=True, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(pitch=-10.0, relative=True, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(roll=-10.0, relative=True, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(roll=20.0, relative=True, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(roll=-10, relative=True, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))


# back to forward
# arm.set_servo_angle(angle=[0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)



# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
