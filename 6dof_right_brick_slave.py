#!/usr/bin/env pybricks-micropython
import sys
from threading import Thread

from pybricks.ev3devices import ColorSensor, Motor, TouchSensor
from pybricks.hubs import EV3Brick
from pybricks.messaging import (BluetoothMailboxClient, NumericMailbox,
                                TextMailbox)
from pybricks.parameters import Color, Port
from pybricks.tools import wait

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# allow to pass master brick name from CLI for remote start
if len(sys.argv) > 1:
    MASTER_BRICK=sys.argv[1]
else:
    # THIS IS THE NAME USED BY THE MASTER EV3 BRICK THIS SLAVE WILL CONNECT TO
    # BY DEFAULT IT WILL BE "ev3dev", IF YOU RENAME IT, CHANGE IT HERE. (highbaycrane)
    MASTER_BRICK='ev3dev'


# Create your objects here.
ev3 = EV3Brick()
roll_head = Motor(Port.A)
yaw_base = Motor(Port.B)
touch_yaw_base = TouchSensor(Port.S1)
color_roll_head = ColorSensor(Port.S2)

roll_head.control.limits(1400,3600,100)
yaw_base.control.limits(1400,1400,100)

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
ev3.speaker.set_volume(volume=80, which='_all_')
ev3.speaker.beep()
ev3.light.on(Color.RED)

client.connect(MASTER_BRICK)


def move_yaw_base():
    while True:
        yaw_base_bt_num.wait_new()
        yaw_base.run_target(yaw_base_bt_sp.read(), yaw_base_bt_num.read(), wait=False)

def move_roll_head():
    while True:
        roll_head_bt_num.wait_new()
        print('received roll head num: {}'.format(roll_head_bt_num.read()))
        print('current angle: {}'.format(roll_head.angle()))
        roll_head.run_target(roll_head_bt_sp.read(), roll_head_bt_num.read(), wait=False)

def control_check():
    if yaw_base.control.done() and roll_head.control.done():
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

if touch_yaw_base.pressed():
    while touch_yaw_base.pressed():
        yaw_base.run(-200)
    wait(500)

while not touch_yaw_base.pressed():
    yaw_base.run(200)
yaw_base.hold()
yaw_base.reset_angle(yaw_base_bt_zeroing.read())


commands_bt_text.send('Initiated yaw base')

while commands_bt_text.read() != 'Initiate roll head':
    wait(100)

# already on red, back off a bit for clean calibration
if color_roll_head.color() == Color.RED:
    while color_roll_head.color() == Color.RED:
        roll_head.run(1400)
    wait(200)

# start calibrating move
while color_roll_head.color() != Color.RED:
    roll_head.run(-600)

if color_roll_head.color() != Color.RED:
    print('DEBUG1: not on red as expected?!')

wait(400)  # determine optimal number for this delay, or figure out how to make it adjust itself correctly
roll_head.hold()
if color_roll_head.color() != Color.RED:
    print('DEBUG2: not on red as expected?!')

roll_head.reset_angle(roll_head_bt_zeroing.read())

commands_bt_text.send('Initiated roll head')

wait(500)
# indicate that we are now calibrated
ev3.light.on(Color.ORANGE)

while True:
    try:
        control_check()
    except OSError:
        print('Looks like we lost connection, shutting down slave...')
        sys.exit(0)

    wait(10)
