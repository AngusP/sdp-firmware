#!/bin/python
import serial
import time

kick_power = -255
kick_time = 400

def main():
    which_port = raw_input("Serial port? (dfl /dev/ttyACM0) ")

    if which_port is '':
        which_port = '/dev/ttyACM0'
    
    port = serial.Serial(which_port, baudrate=115200, timeout=3.0)

    if port is None:
        raise Exception("Couldn't open port " + which_port)

    port.write("kick " + str(kick_power) + " " + str(kick_time) + "\n")
    
if __name__ == '__main__':
    print "Milestone 1 kicker\n"
    try:
        while True:
            main()
    except EOFError:
        exit()
