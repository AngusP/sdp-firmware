#!/bin/python
import numpy as np
import math
import serial
import time
import datetime

move_speed = 10.0
rot_speed = 10.0

def main():
    which_port = raw_input("Serial port? (dfl /dev/ttyACM0) ")

    if which_port is '':
        which_port = '/dev/ttyACM0'
    
    port = serial.Serial(which_port, baudrate=115200, timeout=3.0)

    if port is None:
        raise Exception("Couldn't open port " + which_port)
    
    print "Sending move command... Hit enter to stop"
    #port.write("M 250 -255 0\n")
    #port.write("M -83 -83 -83\n")
    port.write("M 84 84 84\n")
    started = datetime.datetime.now()
    block = raw_input("> ")
    port.write("M 0 0 0\n")
    stopped = datetime.datetime.now()

    tdiff = stopped - started
    print "timedelta was",
    print tdiff

    print "Done, exiting"

    
if __name__ == '__main__':
    print "Milestone 1 turtle motion c&c\n"
    main()
