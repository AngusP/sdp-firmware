#!/bin/python
import numpy as np
import math
import serial
import time

move_speed = 10.0
rot_speed = 10.0

def main():
    which_port = raw_input("Serial port? (e.g. /dev/ttyACM0) ")
    port = serial.Serial(which_port, baudrate=115200, timeout=3.0)

    if port is None:
        raise Exception("Couldn't open port "+which_port)
    
    disp_x = float(raw_input("x (forward) displacement (cm): "))
    disp_y = float(raw_input("y (forward) displacement (cm): "))
    rotation = float(raw_input("rotational difference (degrees, cw): "))

    disp = math.sqrt(pow(disp_x,2) + pow(disp_y,2))
    disp_rot = np.arctan(disp_y / disp_x) * (180.0 / np.pi)

    print "Displacement is",
    print disp
    print "With a rotation of",
    print disp_rot,
    print "degrees"

    move_time = disp / move_speed
    rot_time = disp_rot / rot_speed

    print "Exectuting move..."
    # rotate
    
    # forward
    port.write("M 240 -255 0\n")
    time.sleep(move_time)
    port.write("M 0 0 0\n")

    print "Done, exiting"

    
if __name__ == '__main__':
    print "Milestone 1 turtle motion c&c\n"
    main()
