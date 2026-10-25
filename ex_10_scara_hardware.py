# -*- coding: utf-8 -*-
"""
Created on Thu Oct 22 17:13:50 2026

@author: Admin
"""

# BASIC IMPORTS
import time
import math
import nest_asyncio
nest_asyncio.apply()
from DobotEDU import dobot_edu
edu = dobot_edu
magicbox = edu.magicbox
magicbox.on_pause = lambda: None
print("Searching...")
devices = magicbox.search_dobot()
print(devices)
magicbox._port_name = "COM12"
print("Using port:", magicbox._port_name)
result = magicbox.connect_dobot(queue_start=True, is_queued=False)
print("Connect result:", result)



#%%CALCULATIONS ABOUT THE LINEAR SLIDE
#STEP_PER_REVOLUTION = 360 / step_angle * gear ration * microstepping
STEP_PER_CIRCLE = 360.0 / 1.8 * 5.0 * 16.0
MM_PER_CIRCLE = 3.1415926535898 * 32 #(rotary to linear conversion)

#%% IN_BUILT FUNCTION
magicbox.set_device_withl(enable=True,version=1)
magicbox.set_lspeed_ratio(100)
magicbox.set_ptpwithl_cmd(100)
magicbox.set_device_withl(enable=False,version=1)

#%% Verifying Different Speeds
magicbox.queuedcmd_clear()
magicbox.queuedcmd_start()

PULSES = 3200   # 10 revs at 3200 ppr; adjust to your measured value
for speed in (5000, 10000, 15000, 20000, 25000):
    magicbox.set_emotors(0, True,  speed, PULSES, is_queued=True)   # forward
    time.sleep(1)
    magicbox.set_emotors(0, True, -speed, PULSES, is_queued=True)   # back
    time.sleep(1)
    input(f"speed {speed} done, check the mark, press Enter for next...")
#%%
class Stepper:
    def __init__(self, index, pulses_per_rev=3200, gear_ratio=1.0, speed_dps=180):
        self.index = index
        self.ppr = pulses_per_rev * gear_ratio
        self.speed_dps = speed_dps
        self.angle = 0.0
        self.busy_until = 0.0

    def plan(self, target_deg):
        delta = target_deg - self.angle
        pulses = int(round(abs(delta) / 360.0 * self.ppr))
        return delta, pulses

    def start(self, target_deg, duration=None, margin=0.05):
        """Send the move and return immediately."""
        self.wait()                                   # never overlap own moves
        delta, pulses = self.plan(target_deg)
        if pulses == 0:
            return
        pps = pulses / duration if duration else self.speed_dps / 360.0 * self.ppr
        magicbox.set_emotors(self.index, True, pps if delta > 0 else -pps,
                             pulses, is_queued=False)
        self.busy_until = time.perf_counter() + pulses / pps + margin
        self.angle = target_deg

    def wait(self):
        left = self.busy_until - time.perf_counter()
        if left > 0:
            time.sleep(left)

    def move_to(self, target_deg):
        self.start(target_deg)
        self.wait()


def move_together(*moves, sync_finish=True):
    """moves = (stepper, target_deg), ... Both start together and the call waits for all."""
    duration = None
    if sync_finish:                                   # slowest motor sets the time
        times = []
        for s, tgt in moves:
            _, pulses = s.plan(tgt)
            if pulses:
                times.append(pulses / (s.speed_dps / 360.0 * s.ppr))
        duration = max(times) if times else None
    for s, tgt in moves:
        s.start(tgt, duration)
    for s, _ in moves:
        s.wait()
'''
sync_finish=True: the speeds are scaled so both motors arrive at the same moment. The slower of the two moves is run at its normal speed_dps and the other is slowed to match. With False, each motor runs at its own speed and you wait for the longer one.
No-wait use: call start() on each motor and do other work, then wait() before the next move. start() also waits automatically if that motor is still busy.
Check it: move_together on two equal moves should take about one move’s duration, not two. Time it, and also watch both shafts start together.
Queue: keep the queue empty when using immediate moves, so call magicbox.queuedcmd_clear() once at the start if you used queued mode earlier.
Speed limits: the speed you ask for must be one the motor can actually reach, or it will skip steps while the software believes it arrived.
'''
#%%

#%%
# usage
stp1 = Stepper(index=0, pulses_per_rev=3200, speed_dps=180)
stp1.move_to(90)     # shaft to +90 degrees
stp1.move_to(0)    # shaft to -45 degrees (moves back 135)
#%%
stp2 = Stepper(index=1, pulses_per_rev=3200, speed_dps=180)
stp2.move_to(90)     # shaft to +90 degrees
stp2.move_to(0) 
#%%
stp1 = Stepper(0, pulses_per_rev=3200, speed_dps=180)
stp2 = Stepper(1, pulses_per_rev=3200, speed_dps=180)
move_together((stp1, 90), (stp2, -180))
#%%Reading angles
stp1.angle
#%% READING BASIC SENSORS AND WRITING TO LEDS
buttons = magicbox.get_button_status(port=6)  
#%% Homing in a sequence
magicbox.set_converyor(index=0, enable=True, speed=5)
while True:
    buttons = magicbox.get_button_status(port=6)  
    if buttons[0]==0:
        magicbox.set_converyor(index=0, enable=True, speed=0)
        break
magicbox.set_converyor(index=1, enable=True, speed=5)
while True:
    buttons = magicbox.get_button_status(port=6)  
    if buttons[1]==0:
        magicbox.set_converyor(index=1, enable=True, speed=0)
        break
#%% Simultaneous Homing of Two Axis for Serial SCARA
HOME_SPEED_PPS = 800        # slow, pulses/sec (slower = less overshoot after the switch trips)
CW_SIGN = +1                # +1 or -1: sign of speed that gives clockwise. Verify by test.
TRIGGER_LEVEL = 0           # value the input reads when the switch is pressed (0 or 1)
HOME_TIMEOUT_S = 30         # safety: give up if no switch is hit
POLL_S = 0.002

# limit switch states (True = pressed)
Ls_01 = False               # switch for the motor on stp1
Ls_02 = False               # switch for the motor on stp2

def read_limit_switches():
    """Return (ls1_pressed, ls2_pressed).
    EDIT THIS to match your wiring and the real get_iodi / get_di signature.
    Placeholder below assumes get_iodi(eio_pin) returns 0/1; the pin numbers are examples."""
    v1 = magicbox.get_button_status(port=2)[0]    # <-- eio pin for switch 1 (placeholder)
    v2 = magicbox.get_button_status(port=2)[1]    # <-- eio pin for switch 2 (placeholder)
    return (v1 == TRIGGER_LEVEL), (v2 == TRIGGER_LEVEL)

def stop_motor(stepper):
    magicbox.set_emotor(stepper.index, False, 0, is_queued=False)

def home_both(stp_a, stp_b):
    """Run both steppers clockwise at the same time; each stops when its own switch trips."""
    global Ls_01, Ls_02

    Ls_01, Ls_02 = read_limit_switches()
    done_a, done_b = Ls_01, Ls_02              # already on the switch -> don't move that motor

    spd = CW_SIGN * HOME_SPEED_PPS
    if not done_a:
        magicbox.set_emotor(stp_a.index, True, spd, is_queued=False)
    if not done_b:
        magicbox.set_emotor(stp_b.index, True, spd, is_queued=False)

    t0 = time.perf_counter()
    try:
        while not (done_a and done_b):
            if time.perf_counter() - t0 > HOME_TIMEOUT_S:
                raise TimeoutError("Homing timed out: a limit switch was not reached")

            Ls_01, Ls_02 = read_limit_switches()

            if Ls_01 and not done_a:
                stop_motor(stp_a)
                done_a = True
            if Ls_02 and not done_b:
                stop_motor(stp_b)
                done_b = True

            time.sleep(POLL_S)
    except BaseException:
        stop_motor(stp_a)                      # always stop both on error / Ctrl+C
        stop_motor(stp_b)
        raise

    # declare this position as zero (the Stepper class itself is untouched)
    for s in (stp_a, stp_b):
        s.angle = 0.0
        s.busy_until = 0.0


stp1 = Stepper(0, pulses_per_rev=3200, speed_dps=180)
stp2 = Stepper(1, pulses_per_rev=3200, speed_dps=180)
home_both(stp1, stp2)

#%%
magicbox.disconnect_dobot()

