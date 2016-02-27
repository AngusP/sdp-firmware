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
    magic_number = 0.15
    magic_resolution = 0.2
    
    while t <= (2 * np.pi):
        x = np.cos(t)
        y = np.sin(t)

        print x,
        print " ",
        print y

        t += magic_resolution

        port.write("G " + str(y) + " " + str(x) + " 0\n")

        time.sleep(magic_number)

    t = 0

    while t <= (2 * np.pi):
        x = np.cos(-t)
        y = np.sin(-t)

        print x,
        print " ",
        print y

        t += magic_resolution

        port.write("G " + str(y) + " " + str(x) + " 0\n")

        time.sleep(magic_number)

    port.write("G 0 0 0\n")
        
    
if __name__ == '__main__':
    print "Figure 8 script\n"
    main()
    
