import threading
import time
import csv
from queue import Queue
from dobotapi import Dobot
from dobotapi.dobot_interfaces import GPIO
import time
import sys
sys.path.append(r'C:\SENTH')
from helper_03 import log_robot_data

from queue import Queue
import nest_asyncio
nest_asyncio.apply()
from DobotEDU import dobot_edu

edu = dobot_edu
magicbox = edu.magicbox

magicbox.on_pause = lambda: None

print("Searching...")
devices = magicbox.search_dobot()

print(devices)

magicbox._port_name = "COM6"

print("Using port:", magicbox._port_name)

result = magicbox.connect_dobot(queue_start=True, is_queued=False)

print("Connect result:", result)

PORT = "COM4"
robot = Dobot(PORT)
robot.connect()

#%%
# ============================================================
# ROBOT LOGGER
# ============================================================
from queue import Empty
import math


def move_and_wait(robot, x, y, z, tolerance=1.0, timeout=20):
    print(f"Moving to ({x}, {y}, {z})", flush=True)

    robot.move_to(x, y, z)

    start = time.perf_counter()
    robot_is_paused = False

    while True:
        
        
        # --------------------------------
        # PAUSE
        # --------------------------------
        if pause_event.is_set() and not robot_is_paused:
            print("PAUSING ROBOT", flush=True)
            robot.pause()
            robot_is_paused = True

        # --------------------------------
        # RESUME
        # --------------------------------
       
        if not pause_event.is_set() and robot_is_paused:
            print("RESUMING ROBOT", flush=True)
            robot.resume()
            robot_is_paused = False
            break

        # --------------------------------
        # Position
        # --------------------------------
        pose = robot.get_pose()

        current_x = pose.position.x
        current_y = pose.position.y
        current_z = pose.position.z

        distance = math.sqrt(
            (current_x - x) ** 2 +
            (current_y - y) ** 2 +
            (current_z - z) ** 2
        )

        if distance <= tolerance:
            print("TARGET REACHED", flush=True)
            return

        if time.perf_counter() - start > timeout:
            raise TimeoutError(
                f"Robot failed to reach "
                f"({x}, {y}, {z}) within {timeout}s"
            )

        time.sleep(0.05)
        
# ============================================================
# MOTION THREAD
# ============================================================

def robot_motion_thread(robot):
    from queue import Empty
    robot_is_pause = False
    print("Motion thread started.", flush=True)
    print("Motion: Moving above pickup position", flush=True)
    move_and_wait(robot, 220, 0, 60)
    event_queue.put("Reached Above Pickup Position")
    print("Motion: Moving to pickup position", flush=True)
    move_and_wait(robot, 220, 0, 20)  
    event_queue.put("Reached Pickup Position")   
    print("Motion: Suction ON", flush=True)
    robot.suction_cup.suck()
    event_queue.put("Gripping") 
    print("Motion: Moving to Loc1", flush=True)
    move_and_wait(robot, 220, 0, 60)
    event_queue.put("Reached Loc1")
    print("Motion: Moving to Loc2", flush=True)
    move_and_wait(robot, 260, 120, 60)
    event_queue.put("Reached Loc2")
    
    if proximity_event.is_set() and not robot_is_pause:
        print("PAUSING ROBOT", flush=True)
        robot.pause()
        robot_is_pause = True

    # --------------------------------
    # RESUME
    # --------------------------------
        while True:    
            if not proximity_event.is_set() and robot_is_pause:
                print("RESUMING ROBOT", flush=True)
                robot.resume()
                robot_is_pause = False
                break
                

    print("Motion: Moving to Loc3", flush=True)
    move_and_wait(robot, 260, 120, 20)
    event_queue.put("Reached Loc3")
    print("Motion: Suction OFF", flush=True)
    robot.suction_cup.idle()
    event_queue.put("Reached Loc4")
    print("Motion: Moving to Loc5", flush=True)
    move_and_wait(robot, 260, 120, 60)
    event_queue.put("Reached Loc5")

    


    
    print("===================================", flush=True)
    print("ROBOT MOVEMENT SUCCESSFULLY FINISHED", flush=True)
    print("===================================", flush=True)


    print("Motion thread finished.", flush=True)
    program_running.clear()


#%%
paused=False
# ============================================================
# QUEUES
# ============================================================
event_queue = Queue()
sensing_queue = Queue()
# ============================================================
# THREAD EVENTS
# ============================================================
# Logger runs while this is set
logging_running = threading.Event()

# Overall program running state
program_running = threading.Event()
logging_running.set()
program_running.set()


pause_event = threading.Event()
proximity_event = threading.Event()
# ============================================================
# SHARED SENSOR DATA
# ============================================================
# The MAIN THREAD writes this value.
# The motion thread reads it.
#pot_lock = threading.Lock()
# ============================================================
# SHARED MOTION STATUS
# ============================================================

motion_finished = threading.Event()
motion_error = None
# ============================================================
# START LOGGER
# ============================================================
log_thread = threading.Thread(
    target=log_robot_data,
    kwargs={
        "robot": robot,
        "event_queue": event_queue,
        "logging_running": logging_running,
        "filename": 'dobot_movement_log.csv'
    },
    name="LoggerThread",
    daemon=False
)
# ============================================================
# START MOTION THREAD
# ============================================================
motion_thread = threading.Thread(
    target=robot_motion_thread,
    args=(robot,),
    name="MotionThread",
    daemon=False
)
print("Starting logger thread...", flush=True)

log_thread.start()


print("Starting motion thread...", flush=True)

motion_thread.start()

print(
    "MAIN THREAD: Starting MagicBox sensing",
    flush=True
)
pot = magicbox.get_knob_value(port=3)
vel = min(int(pot/5)+30,100)
robot.speed(vel,100)
while program_running.is_set():

   
    try:
        
        
        but = magicbox.get_button_status(port=2)
        red = but[0]
        blue = but[1]
        
        
        #pir = magicbox.is_pir_detected(port=4)
        #pir=False
        
        proximity = magicbox.get_photoelectric_switch_value(port=1)
        #print('Red: ', red, 'Blue: ', blue, "PUASED_STATUS: ", paused, 'PHOTO: ', proximity)
        if proximity==1:
            proximity_event.set()
        else:
            proximity_event.clear()

        
        if blue == 0 and not paused:
            pause_event.set()
            print("I AM PAUSED", flush=True)
            paused = True
        
        elif blue == 1 and paused:
            pause_event.clear()
            print("I AM RESUMED", flush=True)
            paused = False
        if red==0:
            robot.force_stop()
            time.sleep(0.5)
            logging_running.clear()
            program_running.clear() 
            log_thread.join()
            motion_thread.join()
            
            break       
             
      
    except Exception as e:

        print(
            "MAIN: MagicBox error:",
            repr(e),
            flush=True
        )


    #time.sleep(0.3)






motion_thread.join(timeout=10)
# --------------------------------------------------------
# Tell logger to stop
# --------------------------------------------------------
logging_running.clear()
log_thread.join(timeout=5)

magicbox.disconnect_dobot(queue_stop=True, queue_clear=True, is_queued=False)
print("MagicBox disconnected.",   flush=True)

robot.suction_cup.idle()
robot.close()

