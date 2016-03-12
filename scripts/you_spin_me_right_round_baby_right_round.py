#!/bin/python
import numpy as np
import math
import serial
import time

def main():
    which_port = raw_input("Serial port? (e.g. /dev/ttyACM0) ")

    if which_port is '':
        which_port = '/dev/ttyACM0'
    
    port = serial.Serial(which_port, baudrate=115200, timeout=3.0)

    if port is None:
        raise Exception("Couldn't open port " + which_port)

    #port.write(fwd)
    #time.sleep(move_time)
    
    print "Done, exiting"

    t = 0
    magic_number = 0.05
    magic_resolution = 2.0 * (np.pi / 90.0)
    
    while t <= (2 * np.pi):
        x = -1 * np.sin(t)
        y = np.cos(t)

        print x,
        print " ",
        print y
        
        port.write("G " + str(round(x,4)) + " " + str(round(y,4)) + " -0.39\n")

        t += magic_resolution

        time.sleep(magic_number)

    port.write("G 0 0 0\n")
        
    
if __name__ == '__main__':
    print "Figure 8 script\n"
    main()
    
