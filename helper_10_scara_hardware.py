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
magicbox._port_name = "COM8"
print("Using port:", magicbox._port_name)
result = magicbox.connect_dobot(queue_start=True, is_queued=False)
print("Connect result:", result)

#%%
class Stepper:
    def __init__(self, index, pulses_per_rev=3200, gear_ratio=1, speed_dps=180):
        
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

#%%
# ==========================================
# NEW EXTERNAL MULTI-STEPPER CONTROLLER
# ==========================================
class MultiStepper:
    def __init__(self, stepper1, stepper2):
        self.m1 = stepper1
        self.m2 = stepper2

    def _calculate_natural_duration(self, stepper, target_deg):
        """Calculates the time a stepper needs at its standard speed setting."""
        _, pulses = stepper.plan(target_deg)
        if pulses == 0:
            return 0.0
        # Calculate standard pulses-per-second from the original class logic
        pps = (stepper.speed_dps / 360.0) * stepper.ppr
        return pulses / pps

    def move_simultaneously(self, target_deg1, target_deg2):
        """Moves both motors concurrently, scaling the speeds to finish together."""
        # 1. Clear any pending individual movements first
        self.m1.wait()
        self.m2.wait()

        # 2. Compute individual execution times using original speed parameters
        dur1 = self._calculate_natural_duration(self.m1, target_deg1)
        dur2 = self._calculate_natural_duration(self.m2, target_deg2)

        # 3. Synchronize by choosing the longer window
        max_duration = max(dur1, dur2)

        if max_duration == 0:
            return  # Both motors are already at the target positions

        # 4. Trigger moves back-to-back using your original .start(target, duration) signatures
        # We manually update busy_until *before* the second start to bypass m1's self.wait() rule safely
        self.m1.start(target_deg1, duration=max_duration)
        self.m2.start(target_deg2, duration=max_duration)

        # 5. Block the main thread until both routines finish execution
        self.m1.wait()
        self.m2.wait()

#%% Simultaneous Homing of Two Axis for Serial SCARA
TRIGGER_LEVEL = 0           # value the input reads when the switch is pressed (0 or 1)

# limit switch states (True = pressed)
Ls_01 = False               # switch for the motor on stp1
Ls_02 = False               # switch for the motor on stp2
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
def read_limit_switches(port_id=1):
    """Return (ls1_pressed, ls2_pressed).
    EDIT THIS to match your wiring and the real get_iodi / get_di signature.
    Placeholder below assumes get_iodi(eio_pin) returns 0/1; the pin numbers are examples."""
    v1 = magicbox.get_button_status(port=port_id)[0]    # <-- eio pin for switch 1 (placeholder)
    v2 = magicbox.get_button_status(port=port_id)[1]    # <-- eio pin for switch 2 (placeholder)
    return (v1 == TRIGGER_LEVEL), (v2 == TRIGGER_LEVEL)


#%%HOMING CODE HERE


#%%
# Turns power OFF to Motor 1
magicbox.set_emotors(index=0, enable=False, speed=0, distance=0, is_queued=False)
# Turns power OFF to Motor 2
magicbox.set_emotors(index=1, enable=False, speed=0, distance=0, is_queued=False)

#%%
magicbox.disconnect_dobot()

