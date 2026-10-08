# -*- coding: utf-8 -*-
"""
Created on Tue Oct  6 17:10:03 2026

@author: Admin
"""
import os
import csv
import time
import nest_asyncio
import random
nest_asyncio.apply()
import sys
sys.path.append('C:\SENTH')
from helper_07_dxf import generate_csv_from_dxf
from helper_07_dxf import load_csv
from helper_07_dxf import validate_points
from helper_07_dxf import print_path_information


from DobotEDU import dobot_edu

magician = dobot_edu.magician

# Your robot's COM port
magician._port_name = "COM4"

print("Connecting to Dobot...", flush=True)

result = magician.connect_dobot(
    queue_start=True,
    is_queued=False
)

print("Connect result:", result, flush=True)

#%%
print("Enabling linear rail...", flush=True)

result = magician.set_device_withl(
    enable=True,
    version=1,
    is_queued=False
)

print("Rail enable result:", result, flush=True)
#%%
# ============================================================
# USER SETTINGS
# ============================================================

CSV_FILE = r"C:\SENTH\trajectory_3D.csv"

# CP motion
CP_TARGET_ACC = 20.0
CP_JUNCTION_VEL = 10.0

# CP path velocity.
# Start LOW for your first physical test.
CP_POWER = 10.0

# False = normal CP trajectory
# True  = real-time tracking mode
CP_REALTIME_TRACK = False

# CP acceleration.
# Leave None initially unless you specifically need it.
CP_ACC = None

# CP period.
# Leave None initially unless real-time tracking is used.
CP_PERIOD = None

# Absolute CP mode
CP_ABSOLUTE = 1

# Queue commands
QUEUED = True

# Wait for each individual command?
# IMPORTANT: False allows the CP commands to be queued
# consecutively so the controller can perform continuous path
# planning/look-ahead.
WAIT_FOR_FINISH = False




# ============================================================
# READ CSV
# ============================================================


#%%
# ============================================================
# MAIN
# ============================================================

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

magician.set_cpparams(
    CP_TARGET_ACC,
    CP_JUNCTION_VEL,
    CP_REALTIME_TRACK,
    CP_ACC,
    CP_PERIOD,
    QUEUED
)

# ====================================================
# SEND CONTINUOUS PATH
# ====================================================

print("Sending CP path...")
print("Points:", len(points))
#%%
for i, (x, y, z) in enumerate(points):

    print(
        "CP {:4d}/{:4d}  "
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

    magician.set_cpcmd(
        CP_ABSOLUTE,
        x,
        y,
        z,
        CP_POWER,
        QUEUED,
        WAIT_FOR_FINISH
    )
    time.sleep(1)
print("")
print("All CP commands submitted.")
print("Waiting for path execution...")

# Give the controller time to execute
# the queued trajectory.
#
# We deliberately don't put a delay between
# individual CP commands.

# Estimate a minimum wait.
# Increase this if necessary for a long/slow path.
wait_time = max(5.0, len(points) * 0.02)
time.sleep(wait_time)
print("")
print("CP path submission complete.")


#%%

magician.disconnect_dobot(queue_stop=True, queue_clear=True, is_queued=False)
