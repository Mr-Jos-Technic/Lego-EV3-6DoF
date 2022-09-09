#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, Image, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, LogicMailbox, NumericMailbox, TextMailbox
from threading import Thread
from random import choice
from math import fmod
import sys
import os
import math
import struct

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# MIT License: Copyright (c) 2022 Mr Jos

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~~~~~~~~~6 DEGREE OF FREEDOM~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
##########~LEFT SIDE BRICK WITH PRESET WAYPOINTS PROGRAMMED##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()
#yaw_base = On Right brick Port.B            #Joint number 1(theta1)
pitch_base = Motor(Port.A)                   #Joint number 2(theta2)
pitch_arm = Motor(Port.B)                    #Joint number 3(theta3)
roll_arm = Motor(Port.C)                     #Joint number 4(theta4)
yaw_arm = Motor(Port.D)                      #Joint number 5(theta5)
#roll_head = On Right brick Port.A           #Joint number 6(theta6)
#touch_yaw_base = On Right brick Port.S1     #Touch sensor at back of base for homing:        theta1
touch_pitch_base = TouchSensor(Port.S1)      #Touch sensor at back of the base for homing:    theta2
touch_pitch_arm = TouchSensor(Port.S2)       #Touch sensor at the side of arm for homing:     theta3
touch_roll_arm = TouchSensor(Port.S3)        #Touch sensor at the end of arm for homing:      theta4
#                                            #Stall detection used for for homing:            theta5
#color_roll_head = On Right brick Port.S2    #Color sensor at the front left side for homing: theta6
infra_remote = InfraredSensor(Port.S4)      #OPTIONAL: Infrared sensor for manual movements with a remote

th2_switch = -1  #Switching the direction of the motor for theta2     1/-1
th3_switch = -1  #Switching the direction of the motor for theta3     1/-1


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
yaw_base_zeroing   =   -25   #  -35   6/05/2021  Will be send by BT
pitch_base_zeroing = -1860   # -850  31/01/2021  -1930 on 6/05/2021  -1880 on 8/05/2021
pitch_arm_zeroing  =   550   # -255 originally, -225 on 19/01/2021 530 on 5/05/2021
roll_arm_zeroing   =   885   #  855  31/01/2021
yaw_arm_zeroing    = -1120   #-1120  31/01/2021
roll_head_zeroing  =   -50   #   50   6/05/2021  Will be send by BT

roll_head_zeroing_pitch_base_angle = 33         #Angle in degrees to tip forward to reach the color sensor with the head
roll_head_zeroing_pitch_arm_angle  = 77         #Angle in degrees to tip forward to reach the color sensor with the head ###78###


##########~~~~~~~~~~LENGTH FOR ALL COMPONENTS IN 'mm'~~~~~~~~~~##########
a1 =  165   #Height from floor to center pitch_base [Z]        [175@version1]
a2 =  168   #Height from pitch_base to pitch_arm center [Z]    [145@version1]
a3 =   47   #Height from pitch_arm center to the center of the arm [Z]
a45 = 152   #Length from pitch_arm center to yaw_arm center (Center of wrist!) [X]
a67 = 115   #Length from yaw_arm center (Center of wrist!) to front head center + desired distance in front of the head. [X]
a345 = math.sqrt((a3 ** 2) + (a45 ** 2))     #Diagonal length from pitch_arm center to center of wrist
phi345 = math.degrees(math.atan(a45 / a3))   #Angle for a345 to a3


##########~~~~~~~~~~GEARING~~~~~~~~~~##########
yaw_base_full_rot   =  4200   #theta1   360 /  12 *140  /  12 * 12                           = 1/11.66666
pitch_base_full_rot = 15120   #theta2   360 /  12 * 60  /   8 * 28  /   8 * 24  /  20 * 16   = 1/42
pitch_arm_full_rot  = 10500   #theta3   360 /  12 * 60  /   8 * 28  /  12 * 20               = 1/29.16666
roll_arm_full_rot   =  1800   #theta4   360 /  12 * 60  /  16 * 16  /  16 * 16               = 1/5
yaw_arm_full_rot    =  3600   #theta5   360 /  12 * 60  /  12 * 12  /  16 * 16  /  12 * 24   = 1/10
roll_head_full_rot  =  2520   #theta6   360 /   8 * 56  /  12 * 12  /   4 * 4                = 1/7

yaw_base_gear   = yaw_base_full_rot   / 360
pitch_base_gear = pitch_base_full_rot / 360
pitch_arm_gear  = pitch_arm_full_rot  / 360
roll_arm_gear   = roll_arm_full_rot   / 360
yaw_arm_gear    = yaw_arm_full_rot    / 360
roll_head_gear  = roll_head_full_rot  / 360


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
pitch_base.control.limits(800, 3600, 100)   #800, 3600, 100
pitch_arm.control.limits( 800, 3600, 100)   #800, 3600, 100
roll_arm.control.limits(  500, 3600, 100)   #500, 3600, 100
yaw_arm.control.limits(   800, 3600, 100)   #800, 3600, 100
max_speed = 800   #Override for Maximum speed for all joints. (700)
step = 6          #Distance in mm for each step in Inverse Kinematic mode, lower = more accurate but might start shaking due to slow calculations


##########~~~~~~~~~~MAXIMUM ACCELERATION AND MAXIMUM ANGLE TO SAY A MOVEMENT IS FINISHED~~~~~~~~~~##########
pitch_base.control.target_tolerances(1000, 10)   #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)
pitch_arm.control.target_tolerances( 1000, 10)   #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)
roll_arm.control.target_tolerances(  1000, 10)   #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)
yaw_arm.control.target_tolerances(   1000, 10)   #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)


##########~~~~~~~~~~BLUETOOTH SETUP, SERVER SIDE~~~~~~~~~~##########
server = BluetoothMailboxServer()
commands_bt_text = TextMailbox('commands text', server)              #Main mailbox for sending commands and receiving feedback to other brick
yaw_base_bt_zeroing = NumericMailbox('zero position yaw', server)    #Mailbox for sending theta1 homing position
yaw_base_bt_num = NumericMailbox('yaw base degree', server)          #Mailbox for sending theta1 position
yaw_base_bt_sp = NumericMailbox('yaw base speed', server)            #Mailbox for sending theta1 speed
yaw_base_feedb = NumericMailbox('yaw base feedback', server)         #Mailbox with feedback from current theta1 angle
roll_head_bt_zeroing = NumericMailbox('zero position roll', server)  #Mailbox for sending theta6 homing position
roll_head_bt_num = NumericMailbox('roll head degree', server)        #Mailbox for sending theta6 position
roll_head_bt_sp = NumericMailbox('roll head speed', server)          #Mailbox for sending theta6 speed
roll_head_feedb = NumericMailbox('roll head feedback', server)       #Mailbox with feedback from current theta6 angle


##########~~~~~~~~~~CREATING AND STARTING A TIMER, FOR INVERSE KINEMATIC SMOOTH CONTROL~~~~~~~~~~##########
timer_movement = StopWatch()   #Create timer
#timer_movement.pause()        #Stop the timer (do not stop it use a # infront, this is for debugging only)
timer_movement.reset()         #Put timer back at 0, if not stopped it will just keep running but start from 0 again.
timer_remote = StopWatch()
timer_remote.reset()


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
move_coor_list =   [0, 0, 0, 0, 0, 0]
move_angle_list =  [0, 0, 0, 0, 0, 0]
speed_list =       [0, 0, 0, 0, 0, 0]
old_coor_list =    [0, 0, 0, 0, 0, 0]
old_orientation =  [0, 0, 0]
next_orientation = [0, 0, 0]
max_angle =  0
theta1_rot = 0
theta4_rot = 0
theta6_rot = 0
old_speed = int(max_speed)
tracking = True


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=80, which='_all_')   #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)   #Select speaking language, and a voice (male/female)
small_font = Font(size=6)       # 6 pixel height for text on screen
normal_font = Font(size=10)     #10 pixel height for text on screen
big_font = Font(size=16)        #16 pixel height for text on screen
ev3.screen.set_font(big_font)   #Choose a preset font for writing next texts
ev3.screen.clear()              #Make the screen empty (all pixels white)
ev3.speaker.beep()              #Brick will make a beep sound 1 time
ev3.light.off()                 #Turn the lights off on the brick
ev3.screen.draw_text(4,  2, "yaw base angle: ")     #X/Y position for writing on the screen
ev3.screen.draw_text(4, 20, "pitch base angle: ")   
ev3.screen.draw_text(4, 38, "pitch arm angle: ")    
ev3.screen.draw_text(4, 56, "roll arm angle: ")
ev3.screen.draw_text(4, 74, "yaw arm angle: ")
ev3.screen.draw_text(4, 92, "roll head angle: ")


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########
#Not used right now, it can be used to store coordinates of positions to go to, and call them back when restarting the program
#create_file = open("saveddata.txt", "a")    #Create a file if it does not exist and open it, if it does exist just open it
#create_file.write("")                       #Write "Nothing" to the file to have atleast 1 line in the file
#create_file.close()                         #Close the file again, to be able to call it later again



##########~~~~~~~~~~DEFINE SUB-ROUTINES~~~~~~~~~~##########
##########~~~~~~~~~~Scaling block for changing 0-255 PS4-remote input to range of choice~~~~~~~~~~##########
def scale(val, src, dst):
    return (float(val-src[0]) / (src[1]-src[0])) * (dst[1]-dst[0])+dst[0]


##########~~~~~~~~~~HOLD MOTOR POSITION WHEN MOVEMENT FINISHED~~~~~~~~~~##########
def motor_braking():
    while pitch_base.control.done() != True or pitch_arm.control.done() != True or roll_arm.control.done() != True or yaw_arm.control.done() != True or commands_bt_text.read() != "No movement":
        if pitch_base.control.done() == True: pitch_base.hold()
        if pitch_arm.control.done() == True: pitch_arm.hold()   
        continue


##########~~~~~~~~~~CALCULATE AMOUNT OF STEPS NEEDED TO REACH NEXT END-POINT~~~~~~~~~~##########
def next_coordinate_linear(x_pos, y_pos, z_pos, roll, pitch, yaw, maxspeed):
    global old_orientation
    step_lineair = 1   #Step counter restart at 1st
    motor_braking()
    find_position()    #Perform Forward Kinematics to get current real XYZ position
    distance_list = [x_pos - x_pos_fork, y_pos - y_pos_fork, z_pos - z_pos_fork]                           #XYZ distances
    step_orientation = [roll - old_orientation[0], pitch - old_orientation[1], yaw - old_orientation[2]]   #Orientation distances for roll, pitch, yaw
    
    if max(distance_list) >= math.fabs(min(distance_list)):             #Find the largest displacement from XYZ
        max_distance = max(distance_list)
    else:
        max_distance = math.fabs(min(distance_list))
    if max(step_orientation) >= math.fabs(min(step_orientation)):       #Find the largest twist from any orientation
        max_rotation = max(step_orientation) / 2                        #Orientation movement counts as half distance displacement
    else:
        max_rotation = math.fabs(min(step_orientation)) / 2
    if max_distance < max_rotation:                                     #Find biggest movement to get at the end-point
        max_distance = max_rotation

    sub_steps = math.floor(math.fabs(math.ceil(max_distance / step)))   #Calculate amount of steps needed to reach the given end-point
    
    #Check if the previous end-point has been reached completely yet, if not wait for it
    motor_braking()
    
    #Send each sub-step XYZ position, orientation and maxspeed
    if sub_steps <= 0: print("Position to close to previous end-point")
    else:
        for i in range(-((sub_steps - 1) * step), step, step): #### -1 instead of -2
            next_pos(x_pos - (distance_list[0] / sub_steps) * (sub_steps - step_lineair), y_pos - (distance_list[1] / sub_steps) * (sub_steps - step_lineair), z_pos - (distance_list[2] / sub_steps) * (sub_steps - step_lineair), roll - (step_orientation[0] / sub_steps) * (sub_steps - step_lineair), pitch - (step_orientation[1] / sub_steps) * (sub_steps - step_lineair), yaw - (step_orientation[2] / sub_steps) * (sub_steps - step_lineair), maxspeed)    
            step_lineair += 1
            print(x_pos - (distance_list[0] / sub_steps) * (sub_steps - step_lineair), y_pos - (distance_list[1] / sub_steps) * (sub_steps - step_lineair), z_pos - (distance_list[2] / sub_steps) * (sub_steps - step_lineair), roll - (step_orientation[0] / sub_steps) * (sub_steps - step_lineair), pitch - (step_orientation[1] / sub_steps) * (sub_steps - step_lineair), yaw - (step_orientation[2] / sub_steps) * (sub_steps - step_lineair), maxspeed)


    old_orientation = [roll, pitch, yaw]


##########~~~~~~~~~~CALCULATE THE NEW THETAS FOR 1 POINT~~~~~~~~~~##########
def next_pos(x_pos, y_pos, z_pos, roll, pitch, yaw, maxspeed):
    global max_speed
    global next_orientation
    global theta1_rot
    x_wrist = x_pos - (a67 * math.cos(math.radians(pitch)) * math.cos(math.radians(yaw)))   #Calculate the X-position of the wrist for next point
    y_wrist = y_pos - (a67 * math.cos(math.radians(pitch)) * math.sin(math.radians(yaw)))   #Calculate the Y-position of the wrist for next point
    z_wrist = z_pos - (a67 * math.sin(math.radians(-pitch)))                                #Calculate the Z-position of the wrist for next point
    a15 = math.sqrt(((math.sqrt((x_wrist ** 2) + (y_wrist ** 2))) ** 2) + ((z_wrist - a1) ** 2))   #Calculate distance from center theta2 to center of the wrist
    
    #Find the first 3 thetas
    theta1 = math.degrees(math.atan2(y_wrist, x_wrist))
    theta3 = 180 - math.degrees(math.acos(((a2 ** 2) + (a345 **2) - (a15 ** 2)) / (2 * a2 * a345))) - phi345
    theta2 = math.degrees(math.asin((z_wrist - a1) / a15)) + math.degrees(math.acos(((a2 ** 2) + (a15 ** 2) - (a345 ** 2)) / (2 * a2 * a15))) - 90
    theta32 = theta3 - theta2   #General angle for the pitch
    
    #As the robot moves from one quadrant to another the result will shift 360°, this will compensate for axis 1
    if old_coor_list[0] - (theta1_rot * 360) < -90 and theta1 > 90:
        theta1_rot -= 1
    if old_coor_list[0] - (theta1_rot * 360) > 90 and theta1 < -90:
        theta1_rot += 1
    
    #Find theta5
    theta5 = -math.degrees(math.acos(math.cos(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw))) * math.cos(math.radians(theta32 - pitch))))

    #Find theta4, Inverse Kinematic with quadrants
    if theta1 + (theta1_rot * 360) - yaw > 0 and theta1 + (theta1_rot * 360) - yaw < 180 and pitch != 90 and pitch != -90:
        theta4 = math.degrees(math.acos(-math.sin(math.radians(theta32 - pitch)) * math.cos(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw))) / math.sin(math.radians(theta5))))    #### - pitch! both
    elif theta1 + (theta1_rot * 360) - yaw < 0 and theta1 + (theta1_rot * 360) - yaw > -180 and pitch != 90 and pitch != -90:
        theta4 = math.degrees(math.acos(-math.sin(math.radians(theta32 - pitch)) * math.cos(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw))) / math.sin(math.radians(-theta5)))) + 180
    elif theta32 - pitch > 0:
        theta4 = 0
    elif theta32 - pitch < 0:
        theta4 = 180
    else:
        theta4 = round(roll_arm.angle() / roll_arm_gear, 2)

    #Find theta6, Inverse Kinematic with quadrants
    if theta32 - pitch > 0 and theta5 != 0:
        theta6 = math.cos(math.radians(pitch)) * math.degrees(math.asin(-math.cos(math.radians(theta32 - pitch)) * math.sin(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - math.fmod(yaw, 360)))) / math.sin(math.radians(-theta5)))) + \
            (math.sin(math.radians(pitch)) * (theta1 - yaw)) + roll
    elif theta32 - pitch < 0 and theta5 != 0:
        theta6 = math.cos(math.radians(pitch)) * math.degrees(math.asin(-math.cos(math.radians(theta32 - pitch)) * math.sin(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - math.fmod(yaw, 360)))) / math.sin(math.radians(theta5)))) + \
            180 + (math.sin(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw)) + roll
    elif theta1 - math.fmod(yaw, 360) < 0:
        if math.fmod(yaw, 360) == 0:
            theta6 = 270 + roll
        elif math.fmod(yaw, 360) != 0:
            theta6 = 90 + roll
    elif theta1 - math.fmod(yaw, 360) > 0:
        if math.fmod(yaw, 360) == 0:
            theta6 = 90 + roll
        elif math.fmod(yaw, 360) != 0:
            theta6 = 270 + roll
    else:
        theta6 = round((roll_head_feedb.read() /roll_head_gear) + (round(roll_arm.angle() / roll_arm_gear, 2) / roll_head_gear) - (round((yaw_arm.angle() / yaw_arm_gear) - (round(roll_arm.angle() / roll_arm_gear, 2) / 5), 2) * 4 / roll_head_gear / roll_head_gear), 2) + roll

    #To debug print all values
    #print(theta1, theta2, theta3, theta4, theta5, theta6, roll, pitch, yaw, theta1_rot, theta4_rot, theta6_rot, theta32, x_pos_fork, y_pos_fork, z_pos_fork)

    #check that the values for theta2, theta3 and theta5 are within range of the mechanical capabilities
    possible_angles = "OK"
    if theta2 > 44 or theta2 < -95: ###Max theta2 angles are 44° and -95° always
        possible_angles = "Theta2 out of range"
    elif theta2 > 8 and theta3 < -45 or theta2 <= 0 and theta3 < -78 or theta2 > 0 and theta2 <=8 and theta3 < -78 + 4 * theta2: ###Minimal theta3 angles check
        possible_angles = "Theta3 to small"
    elif theta2 >= -30 and theta3 > 75 or \
        theta2 >= -49 and theta2 < -30 and theta3 > 105 + theta2:
        possible_angles = "Theta3 to big"                            #NOT ALL LIMITS ARE DEFINED, NEED TO ADD MORE LIMITS FOR THETA3 TO PREVENT CRASHES!!!
    elif theta5 > 5 or theta5 < -120:
        possible_angles = "Theta5 out of range"
    
    #Send all thetas to find the correct matching speeds
    if possible_angles == "OK":
        max_speed = int(maxspeed)
        next_orientation = [roll, pitch, yaw]
        calc_speed_motors([theta1, theta2, theta3, theta4, theta5, theta6])  
    else:
        print(possible_angles, theta1, theta2, theta3, theta4, theta5, theta6)
        ev3.speaker.beep()


##########~~~~~~~~~~CALCULATE SPEED FOR EACH MOTOR TO ARRIVE AT THE SAME TIME~~~~~~~~~~##########
def calc_speed_motors(new_coor_list):
    global theta1_rot
    global theta4_rot
    global theta6_rot
    global old_speed
    global old_orientation
    global next_orientation
    
    #As the robot moves from one quadrant to another the result will shift 360°, this will compensate for axis 4 and 6
    if old_coor_list[0] - old_orientation[2] - (theta1_rot * 360) >= 0 and new_coor_list[0] - next_orientation[2] < 0 and new_coor_list[2] - new_coor_list[1] - old_orientation[1] > 0:
        theta4_rot -=  1
    if old_coor_list[0] - old_orientation[2] - (theta1_rot * 360) < 0 and new_coor_list[0] - next_orientation[2] >= 0 and new_coor_list[2] - new_coor_list[1] - old_orientation[1] > 0:
        theta4_rot += 1    
    if old_coor_list[2] - old_coor_list[1] - old_orientation[1] >= 0 and new_coor_list[2] - new_coor_list[1] - next_orientation[1]  < 0 and new_coor_list[0] - next_orientation[2] > 0:
        theta6_rot -= 1
    if old_coor_list[2] - old_coor_list[1] - old_orientation[1] < 0 and new_coor_list[2] - new_coor_list[1] - next_orientation[1] >= 0 and new_coor_list[0] - next_orientation[2] > 0:
        theta6_rot += 1
    new_coor_list[0] += (360 * theta1_rot)
    new_coor_list[3] += (360 * theta4_rot)
    new_coor_list[5] += (360 * theta6_rot)
    
    
    #Calculate how much degrees thetas have to change with realtime current position feedback
    move_coor_list[0] = new_coor_list[0] - old_coor_list[0]
    move_coor_list[1] = new_coor_list[1] - round(pitch_base.angle() * th2_switch / pitch_base_gear, 2)
    move_coor_list[2] = new_coor_list[2] - round(pitch_arm.angle() * th3_switch / pitch_arm_gear, 2)
    move_coor_list[3] = new_coor_list[3] - round(roll_arm.angle() / roll_arm_gear, 2)
    move_coor_list[4] = new_coor_list[4] - round((yaw_arm.angle() / yaw_arm_gear) - (round(roll_arm.angle() / roll_arm_gear, 2) / roll_arm_gear), 2)
    move_coor_list[5] = new_coor_list[5] - old_coor_list[5]

    
    #Save the new orientation as old, preparing for future calculations
    for i in range(3): old_orientation[i] = next_orientation[i]

    #Next line is for debugging, showing time needed to calculate and performing 1 step
    print("Looptime", timer_movement.time(), "Time to execute previous movement", max(move_angle_list) / old_speed * 1000)
    
    #Waiting cycle, calculate the time it takes to perform the previous step and overlap for a smooth movement
    while timer_movement.time() < max(move_angle_list) / old_speed * 1000 - 250: #-250ms overlap (-300 on 29/01/2021)
        continue
    timer_movement.reset()       #Reset the timer to 0
    old_speed = int(max_speed)   #Save current step speed as old speed

    #Calculate how many degrees each motor should turn (absolute)
    move_angle_list[0] = math.fabs(int( move_coor_list[0] * yaw_base_gear))
    move_angle_list[1] = math.fabs(int( move_coor_list[1] * pitch_base_gear))
    move_angle_list[2] = math.fabs(int( move_coor_list[2] * pitch_arm_gear))
    move_angle_list[3] = math.fabs(int( move_coor_list[3] * roll_arm_gear))
    move_angle_list[4] = math.fabs(int((move_coor_list[4] * yaw_arm_gear) + (move_coor_list[3] / roll_arm_gear * yaw_arm_gear)))
    move_angle_list[5] = math.fabs(int((move_coor_list[5] * roll_head_gear) - (move_coor_list[3] * 1) + (move_coor_list[4] * 4 / roll_head_gear)))

    max_angle = max(move_angle_list)   #Find the most degrees a motor has to turn

    if max_angle != 0:
        for i in range(6):                                                          #Looping for the 6 axis
            if int(move_angle_list[i] / max_angle * max_speed) < 20:                #If result for (current axis distance)/(max distance)*(max speed) < 20
                speed_list[i] = 20                                                  #Set the speed to 20, to prevent stalling
            else:
                speed_list[i] = int((move_angle_list[i] / max_angle) * max_speed)   #Else, set current axis speed to the calculated value
            old_coor_list[i] = float(new_coor_list[i])                              #Save current thetas as old thetas, preparing for future calculations

        move_all_motors(new_coor_list[0], new_coor_list[1], new_coor_list[2], new_coor_list[3], new_coor_list[4], new_coor_list[5], \
speed_list[0], speed_list[1], speed_list[2], speed_list[3], speed_list[4], speed_list[5])


##########~~~~~~~~~~SEND ACTUAL MOTOR SPEED AND DESIRED POSITION~~~~~~~~~~##########
def move_all_motors(axis1, axis2, axis3, axis4, axis5, axis6, sp1, sp2, sp3, sp4, sp5, sp6):
    yaw_base_bt_sp.send(  int(max_speed))
    yaw_base_bt_num.send( int(axis1 * yaw_base_gear))
    pitch_base.run_target(sp2, int(axis2 * pitch_base_gear * th2_switch), then=Stop.COAST, wait=False)
    pitch_arm.run_target( sp3, int(axis3 * pitch_arm_gear *  th3_switch), then=Stop.COAST, wait=False)
    roll_arm.run_target(  sp4, int(axis4 * roll_arm_gear), then=Stop.COAST, wait=False)
    yaw_arm.run_target(   sp5, int((axis5 * yaw_arm_gear) + (axis4 / roll_arm_gear * yaw_arm_gear)), then=Stop.COAST, wait=False)
    roll_head_bt_sp.send( int(max_speed*1.5))
    roll_head_bt_num.send(int((axis6 * roll_head_gear) - (axis4 * 1) + (axis5 * 4 / roll_head_gear)))
    


##########~~~~~~~~~~FORWARD KINEMATICS FOR FINDING REALTIME XYZ POSITION~~~~~~~~~~##########
def find_position():
    global x_pos_fork
    global y_pos_fork
    global z_pos_fork
    yaw_base_angle =   math.radians(round(yaw_base_feedb.read() / yaw_base_gear, 2))
    pitch_base_angle = math.radians(round(pitch_base.angle() / pitch_base_gear * th2_switch, 2) + 90)
    pitch_arm_angle =  math.radians(round(pitch_arm.angle() / pitch_arm_gear * th3_switch, 2))
    roll_arm_angle =   math.radians(round(roll_arm.angle() / roll_arm_gear, 2))
    yaw_arm_angle =    math.radians(round((yaw_arm.angle() / yaw_arm_gear) - (roll_arm_angle / roll_arm_gear), 2))
    roll_head_angle =  math.radians(round((roll_head_feedb.read() / roll_head_gear) + (roll_arm_angle / roll_head_gear) - (yaw_arm_angle * 4 / roll_head_gear / roll_head_gear), 2))

    yawbasecos = math.cos(yaw_base_angle)
    yawbasesin = math.sin(yaw_base_angle)
    pitbasecos = math.cos(pitch_base_angle)
    pitbasesin = math.sin(pitch_base_angle)
    pitarmcos  = math.cos(pitch_arm_angle)
    pitarmsin  = math.sin(pitch_arm_angle)
    rolarmcos  = math.cos(roll_arm_angle)
    rolarmsin  = math.sin(roll_arm_angle)
    yawarmcos  = math.cos(yaw_arm_angle)
    yawarmsin  = math.sin(yaw_arm_angle)

    x_pos_fork = round(- a67 * yawbasecos * pitbasecos * pitarmcos * rolarmcos * yawarmsin - a67 * yawbasecos * pitbasesin * pitarmsin * rolarmcos * yawarmsin \
- a67 * yawbasesin * rolarmsin * yawarmsin - a67 * yawbasecos * pitbasecos * pitarmsin * yawarmcos + a67 * yawbasecos * pitbasesin * pitarmcos * yawarmcos \
- a45 * yawbasecos * pitbasecos * pitarmsin + a45 * yawbasecos * pitbasesin * pitarmcos +  a3 * yawbasecos * pitbasecos * pitarmcos \
+  a3 * yawbasecos * pitbasesin * pitarmsin +  a2 * yawbasecos * pitbasecos)
    y_pos_fork = round(- a67 * yawbasesin * pitbasecos * pitarmcos * rolarmcos * yawarmsin - a67 * yawbasesin * pitbasesin * pitarmsin * rolarmcos * yawarmsin \
+ a67 * yawbasecos * rolarmsin * yawarmsin - a67 * yawbasesin * pitbasecos * pitarmsin * yawarmcos + a67 * yawbasesin * pitbasesin * pitarmcos * yawarmcos \
- a45 * yawbasesin * pitbasecos * pitarmsin + a45 * yawbasesin * pitbasesin * pitarmcos +  a3 * yawbasesin * pitbasecos * pitarmcos\
+  a3 * yawbasesin * pitbasesin * pitarmsin +  a2 * yawbasesin * pitbasecos)
    z_pos_fork = round(\
- a67 * pitbasesin * pitarmcos * rolarmcos * yawarmsin + a67 * pitbasecos * pitarmsin * rolarmcos * yawarmsin - a67 * pitbasesin * pitarmsin * yawarmcos \
- a67 * pitbasecos * pitarmcos * yawarmcos - a45 * pitbasesin * pitarmsin - a45 * pitbasecos * pitarmcos +  a3 * pitbasesin * pitarmcos \
-  a3 * pitbasecos * pitarmsin +  a2 * pitbasesin + a1)
    

##########~~~~~~~~~~TRACKING 6DOF BY PS4 REMOTE CONTROLLER OUTPUT~~~~~~~~~~##########
def track_remote():
    while tracking == True:
        print(x_remote, y_remote, z_remote, roll_remote, pitch_remote, yaw_remote)
        next_pos(int(x_remote), int(y_remote), int(z_remote), int(roll_remote), int(pitch_remote), int(yaw_remote), max_speed) 


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_find_position = Thread(target=find_position)   #Create an instance for multiple threads
#sub_find_position.start()                         #Start looping the thread for finding realtime XYZ position in the background (NOT USED)
sub_track_remote = Thread(target=track_remote)


##########~~~~~~~~~~WAIT UNTIL (1) BLUETOOTH DEVICE IS CONNECTED~~~~~~~~~~##########
server.wait_for_connection(1)   #Always start this server-brick first. Then start the slave-brick, or it will timeout
#ev3.speaker.say("Bluetooth connected")   #Make the brick talk (NOT USED)


##########~~~~~~~~~~POSSIBLE MANUAL CONTROL WITH BEACON FOR GOING TO SAFE POSITION FOR HOMING~~~~~~~~~~##########
while True:
    if infra_remote.buttons(1) == []:
        pitch_base.hold()
        pitch_arm.hold()
    elif infra_remote.buttons(1) == [Button.LEFT_UP]: pitch_base.run(200 * th2_switch)
    elif infra_remote.buttons(1) == [Button.LEFT_DOWN]: pitch_base.run(-200 * th2_switch)
    elif infra_remote.buttons(1) == [Button.RIGHT_UP]: pitch_arm.run(-200 * th3_switch)
    elif infra_remote.buttons(1) == [Button.RIGHT_DOWN]: pitch_arm.run(200 * th3_switch)
    elif infra_remote.buttons(1) == [Button.BEACON]: break
pitch_base.hold()
pitch_arm.hold()


##########~~~~~~~~~~WAIT 1 SECOND BEFORE STARTING BLUETOOTH COMMUNICATION~~~~~~~~~~##########
wait(1000)

commands_bt_text.send('Initiate yaw base')   #Send the command for homing theta1
yaw_base_bt_sp.send(800)                     #Send maximal speed for theta1
roll_head_bt_sp.send(800)                    #Send maximal speed for theta6
yaw_base_bt_zeroing.send(int(yaw_base_zeroing))
roll_head_bt_zeroing.send(int(roll_head_zeroing))


##########~~~~~~~~~~EV3 HAS INCREMENTAL ENCODERS IN IT'S MOTORS, SO IT'S NEEDED TO CALIBRATE EVERY STARTUP~~~~~~~~~~##########
##########~~~~~~~~~~HOMING THETA3~~~~~~~~~~##########
if touch_pitch_base.pressed() == True:          #If theta2 is on his touch sensor, make it go up first to avoid a crash
    while touch_pitch_base.pressed() == True:
        pitch_base.run(-400 * th2_switch)
    wait(3000)
pitch_base.hold()

if touch_pitch_arm.pressed() == True:           #If theta3 is pressing its touch sensor, make it move forward until it does not touch anymore
    while touch_pitch_arm.pressed() == True:
        pitch_arm.run(600 * th3_switch)
    wait(100)
    pitch_arm.stop()
    wait(250)

while touch_pitch_arm.pressed() == False:       #Homing theta3
    pitch_arm.run(-600 * th3_switch)
pitch_arm.hold()                                #Active brake theta3
pitch_arm.reset_angle(pitch_arm_zeroing)        #Set motor angle for theta3 to the homing angle

pitch_arm.run_target(800, 0)

##########~~~~~~~~~~HOMING THETA4~~~~~~~~~~##########
if touch_roll_arm.pressed() == True:            #If theta4 is pressing its touch sensor, make it move back until it does not touch anymore
    while touch_roll_arm.pressed() == True:
        roll_arm.run(-200)                      #Turning theta4 will make theta5 and theta6 turn as well
        yaw_arm.run(-80)                        #Turn theta5 at 2.5x smaller speed to avoid a collision
    wait(750)
roll_arm.stop()                                 #Soft brake theta4
yaw_arm.stop()                                  #Soft brake theta5

while touch_roll_arm.pressed() != True:         #Homing theta4
    roll_arm.run(600) #### 200
    yaw_arm.run(240) #### 80
roll_arm.hold()                                 #Active brake theta4
yaw_arm.stop()                                  #Soft brake theta5
roll_arm.reset_angle(roll_arm_zeroing)          #Set motor angle for theta4 to the homing angle


##########~~~~~~~~~~HOMING THETA5~~~~~~~~~~##########
yaw_arm.run_angle(200, (roll_arm_zeroing + (roll_head_full_rot / 4)) / -5, wait=False)   #Turn theta5 to match theta4 rotations
roll_arm.run_target(800, (roll_arm_full_rot) / 4)                                        #Turn theta4 90° so theta5 can be safely zeroed

yaw_arm.run_until_stalled(-600, then=Stop.BRAKE, duty_limit=40)   #Homing theta5 with stall detection, no brake at all after stall
wait(150)                                                         #Wait 150ms for easing theta5
yaw_arm.reset_angle(yaw_arm_zeroing)                              #Set motor angle for theta5 to the homing angle
yaw_arm.run_target(800, - yaw_arm_full_rot / 4)                   #Theta5 go to safe position -90°
roll_arm.run_target(800, 0)                                       #Theta4 go to safe position   0°


##########~~~~~~~~~~HOMING THETA2~~~~~~~~~~##########
while commands_bt_text.read() != 'Initiated yaw base':   #Check if theta1 has finished homing
    continue
yaw_base_bt_num.send(0)                                  #Theta1 go to safe position 0°
pitch_arm.run_target(800, pitch_arm_full_rot / 8 * th3_switch)                             #Theta3 go to safe position 0°

if touch_pitch_base.pressed() == True:          #If theta2 is pressing its touch sensor, make it move back until it does not touch anymore
    while touch_pitch_base.pressed() == True:
        pitch_base.run(-800 * th2_switch)
    wait(700)                                   #Time for running backwards
    pitch_base.stop()                           #Soft brake theta2
    wait(250)

while touch_pitch_base.pressed() == False:      #Homing theta2
    pitch_base.run(400 * th2_switch)
pitch_base.hold()                               #Active brake theta2
pitch_base.reset_angle(pitch_base_zeroing)      #Set motor angle for theta2 to the homing angle




##########~~~~~~~~~~HOMING THETA6~~~~~~~~~~##########
yaw_arm.run_target(800, - yaw_arm_full_rot / 4 - (360 / 4 / roll_arm_gear * yaw_arm_gear), wait=False) 
roll_arm.run_target(800, - roll_arm_full_rot / 4)
pitch_base.run_target(800, - pitch_base_full_rot / 360 * roll_head_zeroing_pitch_base_angle * th2_switch, wait=False)
while True:
    if pitch_base.angle() > 5 and th2_switch == -1 or pitch_base.angle() < 5 and th2_switch == 1:
        pitch_arm.run_target(800, pitch_arm_full_rot / 8 * th3_switch + th3_switch * ((pitch_arm_full_rot / 360 * roll_head_zeroing_pitch_arm_angle - pitch_arm_full_rot / 8) * (pitch_base.angle() / (pitch_base_full_rot / 360 * roll_head_zeroing_pitch_base_angle))), wait=False)
    if pitch_base.control.done() == True:       #Wait for the movement to be finished
        break                                   #Break out of the loop
pitch_arm.run_target(800, pitch_arm_full_rot / 360 * roll_head_zeroing_pitch_arm_angle * th3_switch)

commands_bt_text.send('Initiate roll head')     #Send the command for homing theta6 
while commands_bt_text.read() != 'Initiated roll head':
    continue                                    #Check if theta6 has finished homing
roll_head_bt_num.send(0)                        #Theta6 go to position 0°


##########~~~~~~~~~~ALL THETAS GO TO 0°~~~~~~~~~~##########
pitch_arm.run_target(800, 0, wait=False)
pitch_base.run_target(800, 0)
yaw_arm.run_target(800, 0, wait=False) 
roll_arm.run_target(800, 0)
yaw_base_bt_num.send(0)
find_position()
wait(500)

#ev3.speaker.say("All motor positions at 0 degrees")


#####################################################################
#####################################################################
##########~~~~~~~~~~~~~~~~HOMING ALL JOINTS~~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~~~FINISHED, ALL AT 0°~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~~~~SEND COORDINATES~~~~~~~~~~~~~~~~~##########
#####################################################################
##########~~~~~~~~~~~~~1) INVERSE KINEMATIC~~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~POSITION END-EFFECTOR~~~~~~~~~~~~~~~##########
##########~~~~X Y Z (mm) ROLL PITCH YAW (°) SPEED (°/s)~~~~##########
#####################################################################
##########~~~~~~~~~~~~~2) DIRECT POINT CONTROL~~~~~~~~~~~~~##########
##########~TH1 TH2 TH3 TH4 TH5 TH6 SP1 SP2 SP3 SP4 SP5 SP6~##########
#####################################################################
#####################################################################

# 1) Example             X     Y     Z    Roll  Pitch  Yaw  Speed
#next_coordinate_linear( 270,  100,  250,    0,    0,    0, 700) 

# 2) Example      TH1   TH2   TH3   TH4   TH5   TH6  SP1  SP2  SP3  SP4  SP5  SP6     THETAS in real angles for the joint, SPEED minimal 50 and maximal 800
#move_all_motors(  45,   10,  -20,   70,  -60,  180, 500, 700, 600, 600, 200, 100)
#THETAS [1: +Counter-clockwise from top; 2: +lean backwards; 3: +tip forward; 4: +clockwise from rear; 5: +tip down to zero-point arm; 6: +clockwise from rear]

while True:
    next_coordinate_linear( 270,  100,   220,    0,    0,    0, 700)   
    next_coordinate_linear( 270, -100,   220,    0,    0,    0, 700)   


while True:
    next_coordinate_linear( 154,    0,   68,    0,   90,    0, 700)   
    next_coordinate_linear( 154,  136,   68,    0,   90,    0, 700)
    next_coordinate_linear( 154,  136,   68,   90,   90,    0, 700)
    next_coordinate_linear( -37,  136,   68,   90,   90,    0, 700)
    next_coordinate_linear( -37,  136,   68,    0,   90,    0, 700)
    next_coordinate_linear( -37,  136,   68,    0,    0,    0, 700)
    next_coordinate_linear( 270,  136,   68,    0,    0,    0, 700)
    next_coordinate_linear( 270,  136,  400,    0,    0,    0, 700)
    next_coordinate_linear( 270, -136,  400,    0,    0,    0, 700)
    next_coordinate_linear( 270, -136,  220,    0,    0,    0, 700)
    next_coordinate_linear( 270, -136,  220,    0,    0,  -90, 700)
    next_coordinate_linear(  65, -205,  370,    0,    0,    0, 700)
    next_coordinate_linear(  65, -205,  172,    0,    0,    0, 700)
    next_coordinate_linear( 170, -205,  172,    0,    0,    0, 700)
    next_coordinate_linear( 170, -205,  172,  720,    0,    0, 700)
    next_coordinate_linear( 170, -205,  172,  290,    0,    0, 700)
    next_coordinate_linear( 170, -205,  227,  290,    0,    0, 700)
    next_coordinate_linear( 170, -205,  227,  290,   90,    0, 700)
    next_coordinate_linear( 170, -205,  227,    0,   90,    0, 700)

#Measurement control, move lineair for 10centimeter
next_coordinate_linear(200, 10, 400, 0, 0, 0, 700)
#wait(15000)
next_coordinate_linear(200, 10, 300, 0, 0, 0, 700)
#wait(15000)
next_coordinate_linear(200, 10, 200, 0, 0, 0, 700)
#wait(15000)
next_coordinate_linear(300, 10, 200, 0, 0, 0, 700)
#wait(15000)
next_coordinate_linear(300, 10, 100, 0, 0, 0, 700)
#wait(15000)
next_coordinate_linear(300, 10, 300, 0, 0, 0, 700)
#wait(15000)
next_coordinate_linear(150, 10, 300, 0, 0, 0, 700)
#wait(1000000)


#Showcase 6DoF          X     Y     Z    Roll  Pitch  Yaw  Speed
next_coordinate_linear( 290,  200,  250,    0,    0,   90, 700)   
next_coordinate_linear( 290, -200,  250,    0,    0,  -90, 700)
next_coordinate_linear( 290, -200,  150,    0,    0,    0, 700)
next_coordinate_linear( 290,    0,  150,    0,    0,    0, 700)
next_coordinate_linear( 190,    0,  150,    0,   90,    0, 700)
next_coordinate_linear( 190, -190,  150,    0,   90,    0, 700)
next_coordinate_linear( 270, -190,  200,    0,    0,    0, 700)
next_coordinate_linear( 270, -190,  200,  360,    0,    0, 700)
next_coordinate_linear( 270,  190,  200,    0,    0,    0, 700)


motor_braking()

ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")

