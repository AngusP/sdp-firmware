#!/bin/python
import numpy as np
import math
import serial
import time
import curses
import sys

def main():
    try:
        which_port = sys.argv[1]
    except IndexError:
        which_port = '/dev/ttyACM0'
    
    stdscr = curses.initscr()
    curses.cbreak()
    curses.noecho()
    curses.curs_set(False)
    stdscr.keypad(1)

    port = serial.Serial(which_port, baudrate=115200, timeout=3.0)

    if port is None:
        raise Exception("Couldn't open port " + which_port)

    vector = [0,0,0]
    grab_state = False

    while True:
        # WASD Control for translation, QE for rotate
        stdscr.addstr(0,0,"Hit 'x' to quit")
        stdscr.refresh()
        
        key = stdscr.getch()
        stdscr.clear()
        
        if key == ord('w'):
            # Forward
            stdscr.addstr(1,0,"forward")
            if vector[1] == 0:
                vector[1] = 1
            else:
                vector[1] = 0

        elif key == ord('s'):
            # Backward
            stdscr.addstr(1,0,"backward")
            if vector[1] == 0:
                vector[1] = -1
            else:
                vector[1] = 0

        elif key == ord('a'):
            # Left
            stdscr.addstr(1,0,"left")
            if vector[0] == 0:
                vector[0] = -1
            else:
                vector[0] = 0

        elif key == ord('d'):
            # Right
            stdscr.addstr(1,0,"right")
            if vector[0] == 0:
                vector[0] = 1
            else:
                vector[0] = 0

        elif key == ord('q'):
            # Right
            stdscr.addstr(1,0,"rotate left")
            if vector[2] == 0:
                vector[2] = 1
            else:
                vector[2] = 0

        elif key == ord('e'):
            # Right
            stdscr.addstr(1,0,"rotate right")
            if vector[2] == 0:
                vector[2] = -1
            else:
                vector[2] = 0

        elif key == ord(' '):
            # Kicking
            stdscr.addstr(1,0,"kick")
            port.write("kick\n")

        elif key == ord('\n'):
            # Grabbing
            stdscr.addstr(1,0,"grab")
            if grab_state:
                port.write("grab 0\n")
            else:
                port.write("grab 1\n")
 
            grab_state = not grab_state
            

        elif key == ord('x'):
            curses.nocbreak()
            stdscr.keypad(0)
            curses.echo()
            curses.endwin()
            port.write("G 0 0 0\n")
            return
        
        port.write("G " + 
                   str(vector[0]) + " " + 
                   str(vector[1]) + " " + 
                   str(vector[2]) + "\n")

        stdscr.refresh()
        
    
if __name__ == '__main__':
    main()
    

