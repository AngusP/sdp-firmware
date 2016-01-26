#!/bin/python
import serial
import time

def main(port):

    whocares = int(raw_input("Kick 1) ~150cm, 2) ~100cm or 3) ~50cm? "))

    if whocares not in [1,2,3]:
        raise Exception("RFTM")

    if whocares == 1:
        # 150cm
        port.write("kick -255 400\n")

    if whocares == 2:
        # 100cm
        port.write("kick -180 500\n")

    if whocares == 3:
        # 050cm
        port.write("kick -180 300\n")
        time.sleep(0.5)
        port.write("kick -180 400\n")
    
if __name__ == '__main__':
    print "Milestone 1 kicker"
    print "C-d exits\n"
    
    which_port = raw_input("Serial port? (dfl /dev/ttyACM0) ")

    if which_port is '':
        which_port = '/dev/ttyACM0'

    port = serial.Serial(which_port, baudrate=115200, timeout=3.0)
        
    if port is None:
        raise Exception("Couldn't open port " + which_port)
    
    try:
        while True:
            main(port)
    except EOFError:
        exit()
