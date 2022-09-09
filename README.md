# Lego-EV3-6DoF
The MicroPython programs for my 6Degrees of Freedom Lego robotic arm with Forward and Inverse Kinematics

This is where I will publish all my free to use micropython programs involved in my large automated Lego warehouse. (This one is for the small 6DoF)

This version uses:
- 2x EV3 controller with PyBricks MicroPython
- 2x EV3 large motor
- 4x EV3 medium motor
- 4x EV3 Touch sensor
- 1x EV3 Color sensor
- 5x EV3 25cm cable
- 1x EV3 35cm cable
- 6x EV3 50cm cable

  
Current setup video:

 [![Lego EV3 6Dof by Mr Jos on YouTube](https://img.youtube.com/vi/d_PGFmLfKlA/0.jpg)](https://www.youtube.com/watch?v=d_PGFmLfKlA "Lego EV3 6Dof by Mr Jos on YouTube")


 2 MicroPython programs are running at a time;
- Left brick (master program, has different programs for normal fixed points movements, and a PS4 controlled program)
- Right brick (slave program that is the same for all kind of modes, it just executes the master commands and sends back position feedback)

PS4 controller part written by Anton's Mindstorms (Copyright (c) 2021 Anton's Mindstorms)

All my programs/machines are just made for fun and demonstrating what plastic bricks are capable of doing.  
# Any possible similarities to real machines are purely coincidental
