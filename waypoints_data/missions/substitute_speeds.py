#!/usr/bin/env python

import csv
import sys

if len(sys.argv) != 3:
    print("Usage:")
    print(sys.argv[0] + " waypoints_file desired_speed")
    print(sys.argv[0] + " waypoints.csv 7.0")
    print("")
    print("Note: First and last waypoint will have 0.0 speed")
    exit(0)

filename = sys.argv[1]
wanted_speed = float(sys.argv[2])

print("Reading from: " + str(filename))
print("Setting speeds to: " + str(wanted_speed))
print("Writing to: " + str(filename + ".new_speed.csv"))

with open(filename, 'r') as fin:
    for i, l in enumerate(fin):
        pass
    line_numbers = i
print("File with " + str(i) + " lines")

with open(filename, 'r') as fin:
    r = csv.reader(fin)
    with open(filename + ".new_speed.csv", 'w') as fout:
        w = csv.writer(fout)
        for lno, line in enumerate(r):
            # write header
            if lno == 0:
                w.writerow(line)
                continue
            # first and last waypoint speed 0.0
            elif lno == 1 or lno == line_numbers:
                line[4] = 0.0
            # the rest of the lines on the desired speed
            else:
                line[4] = wanted_speed
            w.writerow(line)

print("Done.")
