# -*- coding: utf-8 -*-
"""
Created on Thu Oct  8 07:17:29 2026

@author: Admin
"""

# version: Python3
from DobotEDU import *
import os
import csv
import time
import sys
sys.path.append('C:\SENTH')
from helper_07_dxf import generate_csv_from_dxf
from helper_07_dxf import load_csv
from helper_07_dxf import validate_points
from helper_07_dxf import print_path_information
CSV_FILE = r"C:\SENTH\trajectory_3D.csv"

print("")
print("DobotEDU CSV Continuous Path")
print("--------------------------------------------")

points = load_csv(CSV_FILE)
for i in range(len(points)):
    #points[i][0]=random.randint(145,155)
    points[i][2] = -45
validate_points(points)

print_path_information(points)

print("Configuring CP parameters...")

for i, (x, y, z) in enumerate(points):

    print(
        "PTP {:4d}/{:4d}  "
        "X={:9.3f}  "
        "Y={:9.3f}  "
        "Z={:9.3f}".format(
            i + 1,
            len(points),
            x,
            y,
            z
        )
    )

    magician.set_ptpwithl_cmd(mode=1,  x=x, y=0,  z=z,  r=0,  l=y )
    time.sleep(1)
print("")
print("All PTP commands submitted.")





















