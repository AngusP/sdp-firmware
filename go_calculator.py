#!/bin/python
import numpy as np

holo_mat = np.array([
    [-1.0/2,   np.sqrt(3)/2.0,   1.0],
    [-1.0/2,  -np.sqrt(3)/2.0,   1.0],
    [1.0,      0.0,              1.0]
]);

def main():
    velo_x = raw_input("x (forward) velocity: ")
    velo_y = raw_input("y (right) velocity: ")
    velo_a = raw_input("rotational speed: ")

    vector = np.array([float(velo_y), float(velo_x), float(velo_a)])

    out_v = holo_mat.dot(vector)

    largest = 0.0
    for x in out_v:
        if abs(x) > abs(largest):
            largest = x

    print "-----"
    print out_v
    print "-----"
    
    out_v = out_v.dot((1.0/abs(largest))*255.0)

    print "G",
    for x in out_v:
        print int(round(x,0)),
    print "\n"
    return vector


if __name__ == '__main__':
    try:
        print "Holonomic Matrix calculator\n"
        while True:
            main()
    except EOFError:
        exit()

