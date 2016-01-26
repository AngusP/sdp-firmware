#!/bin/python
import numpy as np
import serial

def main():
    which_port = raw_input("Serial port? (e.g. /dev/ttyACM0) ")
    port = Serial.serial(which_port, baudrate=115200, timeot=3.0)

    if port is None:
        raise Exception("Couldn't open port "+which_port)
    
    disp_x = float(raw_input("x (forward) displacement (cm): "))
    disp_y = float(raw_input("y (forward) displacement (cm): "))
    rotation = float(raw_input("rotational difference (degrees, cw): "))

    disp = np.sqrt(np.power(disp_x,2) + np.power(disp_x,2))
    disp_rot = np.arctan(disp_y / disp_x) * 360.0

    print "Displacement is",
    print disp
    print "With a rotation of",
    print disp_rot,
    print "degrees"

    
    
if __name__ == '__main__':
    print "Milestone 1 turtle motion c&c\n"
    main()
