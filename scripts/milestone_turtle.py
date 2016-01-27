#!/bin/python
import numpy as np
import math
import serial
import time

cw_rot = "M -83 -83 -83\n"
acw_rot = "M 84 84 84\n"
fwd = "M 250 -255 0\n"

# Moves forward at 29 cm/s
move_speed = 33.79
move_spool = 0.2
# rotational speed, 360 deg in 4.8 sec
rot_speed = 93.9
rot_spool = 0.3

def main():
    which_port = raw_input("Serial port? (e.g. /dev/ttyACM0) ")

    if which_port is '':
        which_port = '/dev/ttyACM0'
    
    port = serial.Serial(which_port, baudrate=115200, timeout=3.0)

    if port is None:
        raise Exception("Couldn't open port " + which_port)
    
    disp_x = float(raw_input("x (forward) displacement (cm): "))
    disp_y = float(raw_input("y (right) displacement (cm): "))
    rotation = float(raw_input("rotational difference (degrees, cw): "))

    disp = math.sqrt(pow(disp_x,2) + pow(disp_y,2))

    if disp_x == 0:
        disp_rot = (disp_y/abs(disp_y)) * 90.0
    elif disp_y == 0:
        if disp_x < 0:
            disp_rot = 180.0
        else:
            disp_rot = 0.0
    elif disp_y < 0 and disp_x > 0:
        # quadrant 2
        disp_rot = (np.arctan(disp_y / disp_x) * (180.0 / np.pi)) + 180.0
    elif disp_y < 0 and disp_x < 0:
        # quadrant 3
        disp_rot = (np.arctan(disp_y / disp_x) * (180.0 / np.pi)) + 180.0
    elif disp_y > 0 and disp_x < 0:
        # quadrant 2
        disp_rot = (np.arctan(disp_y / disp_x) * (180.0 / np.pi)) + 360.0
    else:
        # quadrant 1
        disp_rot = (np.arctan(disp_y / disp_x) * (180.0 / np.pi))

    # important to do this before we decide on cw or acw
    if rotation != 0:
        rotation -= disp_rot
    else:
        # go back to original heading if no rotation wanted
        rotation = -disp_rot
        
    # go ACW if it's shorter
    if disp_rot > 180.0:
        disp_rot = 180.0 - disp_rot

    if rotation > 180.0:
        rotation = 180.0 - rotation
        
    print "Displacement is",
    print disp
    print "With a rotation of",
    print disp_rot,
    print "degrees",
    print "followed by a rotation of",
    print rotation,
    print "degrees"

    move_time = (disp / move_speed) + move_spool
    rot_time = (abs(disp_rot) / rot_speed) + rot_spool

    if rotation != 0:
        final_rot_time = (abs(rotation) / rot_speed) + rot_spool
    else:
        final_rot_time = 0
        
    print "Timings are"
    print move_time,
    print "for movement and ",
    print rot_time,
    print "for initial rotation and finally",
    print final_rot_time
    
    print "Exectuting first rotate move..."
    # rotate to point correct dir, if we have to
    if disp_rot != 0.0:

        if disp_rot < 0:
            # Anti-clockwise
            port.write(acw_rot)
        else:
            # Clockwise
            port.write(cw_rot)

        time.sleep(rot_time)
        # Stop so motors have the same stall torque to overcome
        port.write("M 0 0 0\n")

    
    time.sleep(1)
    
    # forward
    print "Exectuting linear move..."
    port.write(fwd)
    time.sleep(move_time)
    port.write("M 0 0 0\n")

    time.sleep(1)
    print "Executing final rotation..."

    if rotation != 0:

        if rotation < 0:
            port.write(acw_rot)
        else:
            port.write(cw_rot)

        time.sleep(final_rot_time)
        port.write("M 0 0 0\n")
            

    print "Done, exiting"

    
if __name__ == '__main__':
    print "Milestone 1 turtle motion c&c\n"
    main()
