#!/bin/python
import numpy as np
import math
import serial
import time
from get_rotation_delta import get_rotation_delta

fwd = "M 253 -255 0\n"

# Moves forward at 29 cm/s
move_speed = 33.13
move_spool = 0.2

rot_wait = 3

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

    if disp_x == 0 and disp_y != 0:
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

    # If we're doing more than a full circle, don't shorten
    if rotation > 180.0 and rotation < 360:
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
        
    print "Timing for movement is"
    print move_time,
    print "seconds"
    
    # rotate to point correct dir, if we have to
    if disp_rot != 0.0:
        print "Exectuting first rotate move..."
        
        if disp_rot < 0:
            # Anti-clockwise
            port.write("rotate 100 "+ str(get_rotation_delta(disp_rot)) +"\n")
        else:
            # Clockwise
            port.write("rotate -100 "+ str(get_rotation_delta(disp_rot)) +"\n")

        time.sleep(disp_rot / 80.0) # magic constant wooo
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
            port.write("rotate 100 "+ str(get_rotation_delta(rotation)) +"\n")
        else:
            port.write("rotate -100 "+ str(get_rotation_delta(rotation)) +"\n")
    
    print "Done, exiting"

    
if __name__ == '__main__':
    print "Milestone 1 turtle motion c&c\n"
    main()
