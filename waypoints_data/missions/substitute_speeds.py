#!/usr/bin/env python

import csv
import sys

filename = sys.argv[1]
wanted_speed = sys.argv[2]

with open(filename, 'r') as fin:
    for i, l in enumerate(fin):
        pass
    line_numbers = i
print("file with " + str(i) + " lines")

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
