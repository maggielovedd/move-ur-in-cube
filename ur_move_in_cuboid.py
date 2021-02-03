#!/usr/bin/env python
# -*- coding: UTF-8 -*-

'''
Written by Maggie Xu 03 July

This script aims to provide a convinent way to control UR (using urx with python 2.7).

The flow goes like this:
1. It partition the workspace in x,y,z direction (as specify by the user)
2. The user specify the partition to go, then it calculate the center position of that partition
3. execute the position, the orientation is assumed constant
'''

import sys 
import urx
import cv2
import numpy as np
import time
import copy
import rospy
import logging
from math import pi
import math
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# convert euler to quaternion
def euler_to_quaternion(Yaw, Pitch, Roll):

  yaw   = Yaw   * pi / 180 
  pitch = Roll  * pi / 180 
  roll  = Pitch * pi / 180 

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

# convert euler pitch to rotation vector
def euler_to_rotationVector(yaw , roll, pitch):
    nums = euler_to_quaternion(yaw , roll, pitch)
    
    R = np.array([[0,0,0],[0,0,0],[0,0,0]],dtype=float)
    x = float(nums[0])
    y = float(nums[1])
    z = float(nums[2])
    w = float(nums[3]) 

    #calculate element of matrix
    R[0][0] = np.square(w) + np.square(x) - np.square(y) - np.square(z)
    R[0][1] = 2*(x*y + w*z)
    R[0][2] = 2*(x*z - w*y)
    R[1][0] = 2*(x*y - w*z)
    R[1][1] = np.square(w) - np.square(x) + np.square(y) - np.square(z)
    R[1][2] = 2*(w*x + y*z)
    R[2][0] = 2*(x*z + w*y)
    R[2][1] = 2*(y*z - w*x)
    R[2][2] = np.square(w) - np.square(x) - np.square(y) + np.square(z)
    rotationVector  = cv2.Rodrigues(R)

    print "yaw, roll and pitch:"
    print rotationVector[0]
    return rotationVector[0]

# robot go all up back to the straight status
def goto_straight_status(robot):
    print "\n============ Press `Enter` to go back straight-up status ============"
    raw_input()
    v = 0.01
    a = 0.01

    joint = []
    joint.append(0)
    joint.append(-pi/2)
    joint.append(0)
    joint.append(-pi/2)
    joint.append(0)
    joint.append(0)
    robot.movej(joint, acc=a, vel=v)

    print "Robot current joint positions:"
    print robot.getj()

# move to single point with input XYZRPY
def move_to_singleXYZRPY(robot, x, y, z, roll=90, pitch=180, yaw=0):
    print "\n============ Press `Enter` to move to certain XYZRPY ============"
    raw_input()

    rotation = euler_to_rotationVector(yaw, roll, pitch) # yaw->roll->pitch
    pose = []
    pose.append(x)              #px
    pose.append(y)              #py
    pose.append(z)              #pz
    pose.append(rotation[0][0]) #rx
    pose.append(rotation[1][0]) #ry
    pose.append(rotation[2][0]) #rz
    robot.movel(pose, acc=0.005, vel=0.06)

# move to single point with input pose
def move_to_singlePose(robot, pose): 
    print "\n============ Press `Enter` to move to certain pose ============"
    print "\nGo or not(y/n):"

    ans = raw_input()

    if ans == "y":
        robot.movel(pose, acc=0.005, vel=0.02)
    else:
        print "Target cancelled"

# test different positons to find out the workspace of UR
def test_positions(robot):
    move_to_singleXYZRPY(robot, 0, -0.3, 0.5, 90, 180, 0)  #x, y, z, roll, pitch, yaw
    move_to_singleXYZRPY(robot, 0, -0.5, 0.5, 90, 180, 0)  #x, y, z, roll, pitch, yaw
    move_to_singleXYZRPY(robot, 0, -0.3, 0.5, 90, 180, 0)  #x, y, z, roll, pitch, yaw
    move_to_singleXYZRPY(robot, 0, -0.3, 0.4, 90, 180, 0)  #x, y, z, roll, pitch, yaw
    move_to_singleXYZRPY(robot, 0, -0.3, 0.3, 90, 180, 0)  #x, y, z, roll, pitch, yaw        
    move_to_singleXYZRPY(robot, 0.1, -0.3, 0.3, 90, 180, 0)  #x, y, z, roll, pitch, yaw 
    move_to_singleXYZRPY(robot, 0.2, -0.3, 0.3, 90, 180, 0)  #x, y, z, roll, pitch, yaw 
    move_to_singleXYZRPY(robot, 0.2, -0.4, 0.3, 90, 180, 0)  #x, y, z, roll, pitch, yaw 
    move_to_singleXYZRPY(robot, 0.1, -0.2, 0.4, 90, 180, 0)  #x, y, z, roll, pitch, yaw 
    move_to_singleXYZRPY(robot, 0, -0.3, 0.5, 90, 180, 0)  #x, y, z, roll, pitch, yaw
    goto_straight_status(robot)


# calculate corresponding xyz location for each small partition
def locations_xyz(range, partition):  # range is the range for xyz axles in the workspcae(simplification) 
                                      # partition = number of cut

    diff = range[1] - range[0]
    increase = diff / (partition * 2)

    center_location = []

    i = 0

    while i < partition:
        value = range[0] + increase * (2 * i + 1)
        center_location.append(value)
        i += 1

    return center_location

# calculate center points in each small cuboid
def get_center_points(x_range, y_range, z_range, x_partition, y_partition, z_partition):

    # get center location in x,y,z
    location_x = locations_xyz(x_range, x_partition)
    location_y = locations_xyz(y_range, y_partition)
    location_z = locations_xyz(z_range, z_partition)

    center_list = []
    k = 0

    while k < z_partition:
        j = 0
        while j < y_partition:
            i = 0
            while i < x_partition:
                location = [location_x[i], location_y[j], location_z[k]] #x,y,z
                rounded_location = [round(num, 2) for num in location]
                center_list.append(rounded_location)
                i += 1
            j += 1
        k += 1

    return center_list

# plot the workspace cuboid and the target point
def plot_cube(target_list, x, y, z, dx, dy, dz, color='red'):

    plt.close("all")
    fig = plt.figure()
    ax = Axes3D(fig)

    #plot cube
    xx = [x, x, x + dx, x + dx, x]
    yy = [y, y + dy, y + dy, y, y]
    kwargs = {'alpha': 1, 'color': color}
    ax.plot3D(xx, yy, [z] * 5, **kwargs)
    ax.plot3D(xx, yy, [z + dz] * 5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z + dz], **kwargs)
    ax.plot3D([x, x], [y + dy, y + dy], [z, z + dz], **kwargs)
    ax.plot3D([x + dx, x + dx], [y + dy, y + dy], [z, z + dz], **kwargs)
    ax.plot3D([x + dx, x + dx], [y, y], [z, z + dz], **kwargs)

    #plot the target point
    previous_target = target_list[-2]
    target = target_list[-1]
    ax.scatter(previous_target[0], previous_target[1], previous_target[2], marker="o", color='c')
    ax.scatter(target[0], target[1], target[2], marker="v")   # color='y'

    ax.set_zlabel('Z')
    ax.set_ylabel('Y')
    ax.set_xlabel('X')

    plt.title('Workspace')
    plt.show(block=False)

# move the robot in the cuboid workspace
def move_in_cuboid():

    x_range     = [-0.3, 0.2] # unit: m
    y_range     = [-0.8, -0.577]
    z_range     = [0.15, 0.6]
    x_partition = 6
    y_partition = 6
    z_partition = 6
    dx          = x_range[1] - x_range[0]
    dy          = y_range[1] - y_range[0]
    dz          = z_range[1] - z_range[0]

    print "\n=================================================" 
    print "\nx_range=" + str(x_range) + "\ty_range=" + str(y_range) + "\tz_range=" + str(z_range)
    print "\nx_partition=%d   \ty_partition=%d   \tz_partition=%d"%(x_partition, y_partition, z_partition)

    # rospy.loginfo(x_range)

    center_list = get_center_points(x_range, y_range, z_range, x_partition, y_partition, z_partition)
    target_list = []
    target_list.append([0, -0.527, 0.528])

    while True:

        print "\n=================================================" 
        print "Partition x = %d y = %d z = %d"%(x_partition, y_partition, z_partition)
        print "enter q to quit" 
        print "enter c to change partition"
        print "location to move (xyz):"

        num = raw_input()

        if num == 'q':
            print "\n=================================================" 
            print "Bye"
            break

        elif num == 'c':
            print "pls input the new partition in the order of x,y,z:"
            new_p = raw_input()
            x_partition = int(new_p[0])
            y_partition = int(new_p[1])
            z_partition = int(new_p[2])
            print "\nx_range=" + str(x_range) + "\ty_range=" + str(y_range) + "\tz_range=" + str(z_range)
            print "\nx_partition=%d   y_partition=%d   z_partition=%d"%(x_partition, y_partition, z_partition)

            center_list = get_center_points(x_range, y_range, z_range, x_partition, y_partition, z_partition)

        elif 0 < int(num[0]) <= x_partition and 0 < int(num[1]) <= y_partition and 0 < int(num[2]) <= z_partition:
            print "\nCorrect input! Processing"
            index = int(num[0]) + (int(num[1])-1) * x_partition + (int(num[2])-1) * x_partition * z_partition - 1
            target = center_list[index]
            target_list.append(target)
            print "UR will move to" + str(target)

            plot_cube(target_list, x_range[0], y_range[0], z_range[0],
                             dx, dy, dz, color='red')

	    e = raw_input("Go to the target? (enter y for yes)")
		
	    if e == 'y':
		move_to_singleXYZRPY(robot, target[0], target[1], target[2])
	    else: 
		print "target cancelled"

        else:
            print "Wrong input! Pls try again."


if __name__ == "__main__":

    try:  
        logging.basicConfig(level=logging.WARN)
        # robot = urx.Robot("localhost") # for simulation
	    robot = urx.Robot("192.168.0.2") # for real robot
        rospy.init_node('ur_move_in_cuboid', anonymous=True)
        pose = robot.getl()

        #for testing different positions and find the range for x,y,z
        # test_positions(robot)

	user = raw_input("\nGo back to straight?(enter y for yes)")

	if user == 'y':
		goto_straight_status(robot)
	else: 
		print "not going back"
        move_to_singleXYZRPY(robot, 0, -0.527, 0.528, 90, 180, 0)  #x, y, z, roll, pitch, yaw
        print "start move in cube"
        move_in_cuboid()

    finally:
        robot.close() 

