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

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.



# Create your objects here.
ev3 = EV3Brick()
roll_head = Motor(Port.A)
yaw_base = Motor(Port.B)
touch_yaw_base = TouchSensor(Port.S1)
color_roll_head = ColorSensor(Port.S2)

roll_head.control.limits(1400,3600,100)
yaw_base.control.limits(1400,1400,100)

timer1 = StopWatch()
timer1.pause()
timer1.reset()

small_font = Font(size=6)
normal_font = Font(size=10)
big_font = Font(size=16)

client = BluetoothMailboxClient()
commands_bt_text = TextMailbox('commands text', client)
roll_head_bt_zeroing = NumericMailbox('zero position roll', client)
roll_head_bt_num = NumericMailbox('roll head degree', client)
roll_head_bt_sp = NumericMailbox('roll head speed', client)
roll_head_feedb = NumericMailbox('roll head feedback', client)
yaw_base_bt_zeroing = NumericMailbox('zero position yaw', client)
yaw_base_bt_num = NumericMailbox('yaw base degree', client)
yaw_base_bt_sp = NumericMailbox('yaw base speed', client)
yaw_base_feedb = NumericMailbox('yaw base feedback', client)


# Write your program here.
ev3.speaker.beep()
ev3.light.off()
ev3.speaker.set_volume(volume=80, which='_all_')
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)

#create_file = open("saveddata.txt", "a")
#create_file.write("")
#create_file.close()


client.connect('left6dof') ############# THIS IS THE NAME USED BY THE MASTER EV3 BRICK THIS SLAVE WILL CONNECT TO, BY DEFAULT IT WILL BE "ev3dev", IF YOU RENAME IT, CHANGE IT HERE. (highbaycrane)


def move_yaw_base():
    global yaw_base_bt_num
    global yaw_base
    while True:
        yaw_base_bt_num.wait_new()
        yaw_base.run_target(yaw_base_bt_sp.read(), yaw_base_bt_num.read(), wait=False)

def move_roll_head():
    global roll_head_bt_num
    global roll_head
    while True:
        roll_head_bt_num.wait_new()
        print(roll_head_bt_num.read())
        roll_head.run_target(roll_head_bt_sp.read(), roll_head_bt_num.read(), wait=False)

def control_check():
    if yaw_base.control.done() == True and roll_head.control.done() == True:
        commands_bt_text.send("No movement")
        wait(100)
    else:
        commands_bt_text.send("Moving")
        wait(100)

def angle_feedback():
    while True:
        yaw_base_feedb.send(yaw_base.angle())
        roll_head_feedb.send(roll_head.angle())
        wait(250)


sub_yaw_base = Thread(target=move_yaw_base)
sub_roll_head = Thread(target=move_roll_head)
sub_angle_feedback = Thread(target=angle_feedback)
sub_yaw_base.start()
sub_roll_head.start()
sub_angle_feedback.start()


while commands_bt_text.read() != 'Initiate yaw base':
    wait(100)

if touch_yaw_base.pressed() == True:
    while touch_yaw_base.pressed() == True:
        yaw_base.run(-200)
    wait(500)

while touch_yaw_base.pressed() != True:
    yaw_base.run(200)
yaw_base.hold()
yaw_base.reset_angle(yaw_base_bt_zeroing.read())


commands_bt_text.send('Initiated yaw base')

while commands_bt_text.read() != 'Initiate roll head':
    wait(100)

if color_roll_head.color() == Color.RED:
    while color_roll_head.color() == Color.RED:
        roll_head.run(1400)
    wait(200)

while True:
    while color_roll_head.color() != Color.RED:
        roll_head.run(-600)
    roll_head.hold()
    wait(50)
    if color_roll_head.color() != Color.RED:
        continue
    wait(50)
    if color_roll_head.color() != Color.RED:
        continue
    wait(50)
    if color_roll_head.color() != Color.RED:
        continue
    wait(50)
    if color_roll_head.color() != Color.RED:
        continue
    break 

roll_head.reset_angle(roll_head_bt_zeroing.read())

commands_bt_text.send('Initiated roll head')

wait(500)

while True:
    control_check()
    wait(10)





###For debugging use###
yaw_base.run_target(800, 0)
wait(500)
yaw_base.run_target(800, -1050)
yaw_base.run_target(800, 1050)
yaw_base.run_target(800, -200)

