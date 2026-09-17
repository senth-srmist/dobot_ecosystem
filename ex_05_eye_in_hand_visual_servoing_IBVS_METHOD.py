# version: Python3
from DobotEDU import *
magician.set_converyor(index=magician.Stepper1,enable=True,speed=-10.0)
import cv2
import numpy as np
import threading
import queue
import time

# -*- coding: utf-8 -*-
"""
Created on Wed Sep 16 09:25:54 2026

@author: Admin
"""

import cv2
import numpy as np
import threading
import queue
import time

def vision_processing(video_source):
    cap = cv2.VideoCapture(video_source)
    threshold = 40
    while True:
        frame_detections = []
        ret, frame = cap.read()        
        if not ret:
          break
        frame = frame[140:,:,:]
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        b = frame[:, :, 0].astype(np.int16)
        g = frame[:, :, 1].astype(np.int16)
        r = frame[:, :, 2].astype(np.int16)
        
        # 2. Calculate the maximum and minimum values across channels for every single pixel
        max_val = np.maximum(np.maximum(r, g), b)
        min_val = np.minimum(np.minimum(r, g), b)
        
        # 3. Find the spread (difference between the highest and lowest channel)
        channel_diff = max_val - min_val
        
        # 4. Create a boolean mask where the difference is less than or equal to the threshold
        # If the spread is small, no single channel dominates (it's white, grey, black, or glare)
        glare_mask = channel_diff <= threshold
        
        # 5. Clone the frame and force those masked pixels to pure black (0, 0, 0)
        cleaned_frame = frame.copy()
        cleaned_frame[glare_mask] = [0, 0, 0]
        
        gray = cv2.cvtColor(cleaned_frame,cv2.COLOR_RGB2GRAY)
        #hist = cv2.calcHist([gray],[0],None, [256], [0,256])
        thresh, bw = cv2.threshold(gray, 50, 255, type=cv2.THRESH_BINARY)   
       
    
        contours, _ = cv2.findContours(
                bw,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )
    
        for c in contours:
            area = cv2.contourArea(c)
            if area < 5000:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
    
            u = M["m10"] / M["m00"]
            v = M["m01"] / M["m00"]
# Package data tightly into a dictionary or tuple
            
    
    
            center = (int(u), int(v))
            print(center)
            # All pixels inside and on the contour boundary
            mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.drawContours(mask, [c], -1, 255, cv2.FILLED)
        
            pixels = cleaned_frame[mask == 255]
        
            mean_R, mean_G, mean_B = np.mean(pixels, axis=0)
            mean_list = [mean_R, mean_G, mean_B]
            max_index = mean_list.index(max(mean_list))
            RGB_list = ['Red','Green','Blue']
            global_mean = (mean_R+mean_G+mean_B)/3
            color_id=RGB_list[max_index]
            print('Color is:', color_id, "Center:", center, "R:", round(mean_R), "G:", round(mean_G), "B:", round(mean_B))    
            frame_detections.append({"color": color_id, "x": u, "y": v })
            
            text = str(mean_R)+','+str(mean_G)+','+str(mean_B)
            position =  (center[0]-40,center[1]) # X=50, Y=100
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 2
            color = (0, 0, 0)  # Green in BGR
            thickness = 2
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # Draw the text on the image
            cv2.putText(frame, color_id, position, font, scale, color, thickness, cv2.LINE_AA)
            cv2.circle(frame, center, 50, (0, 0, 0), thickness=10, lineType=None, shift=None)
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
          break
        if frame_detections:
              try:
                  # If a previous unread frame is sitting in queue, clear it out.
                  # This guarantees zero lag or buildup if the main thread delays.
                  data_queue.put_nowait(frame_detections)
              except queue.Full:
                  try:
                      data_queue.get_nowait() # Drop stale data
                      data_queue.put_nowait(frame_detections) # Push fresh data
                  except queue.Empty:
                      pass
    cap.release()
    cv2.destroyAllWindows() 
# --- CONFIGURATIONS ---

# ==========================================================
# CONFIGURATION
# ==========================================================
FRAME_WIDTH = 640
IMAGE_CENTER_X = FRAME_WIDTH // 2
DEADZONE = 8
# ==========================================================
# IBVS PARAMETERS
# ==========================================================
# Visual servoing gain
LAMBDA = 0.8
# Initial image Jacobian
## Units:
# pixels / mm
## IMPORTANT:
# Determine the correct sign experimentally.
#
JACOBIAN_INITIAL = 1.0
# Jacobian low-pass filtering
#
# Smaller value = more filtering
# Larger value = faster adaptation
#
JACOBIAN_ALPHA = 0.2
# Minimum rail movement required
# before updating Jacobian
MIN_DELTA_L_FOR_JACOBIAN = 1.0
# Prevent excessively large rail commands
MAX_DELTA_L = 8.0
# Jacobian safety limits
MIN_JACOBIAN = 0.05
MAX_JACOBIAN = 10.0
# Physical limits of rail
RAIL_MIN = 300
RAIL_MAX = 700

# 1. Shared Thread-Safe Queue
# Setting maxsize=1 ensures the main thread ALWAYS gets the freshest frame data.
# If the main thread falls behind, old coordinates are instantly dropped (No Latency buildup).
data_queue = queue.Queue(maxsize=1)
# Flag to cleanly shut down threads
running = True


init_pose = magician.get_pose()
print(init_pose)
fixed_x = 160
eject_x = 260
fixed_y = 0
fixed_z = -10
fixed_r = 0

home_l = 300

magician.set_device_withl(enable=True, version=1)
magician.set_ptpl_params(vel=100, accel=100)
magician.set_ptpwithl_cmd(mode=1, x=fixed_x,y=fixed_y, z=fixed_z, r=fixed_r, l=home_l)
current_l = magician.get_posel()['positionL']  # Current rail position (mm)
# Spin up the background vision thread
detector_thread = threading.Thread(target=vision_processing, args=(0,)) # 0 for webcam
detector_thread.daemon = True # Dies automatically if main thread exits
detector_thread.start()

# ==========================================================
# IBVS VARIABLES
# ==========================================================
previous_x = None
previous_l = current_l
# Current Jacobian estimate
J_image = JACOBIAN_INITIAL
ejected = False
# ==========================================================
# MAIN LOOP
# ==========================================================
print("Main thread running. Listening for tracking data...")
try:
    while running:
        try:
            detections = data_queue.get(timeout=0.01)
            detected_x = detections[0]['x']
            error_x = (detected_x - IMAGE_CENTER_X)
            # ------------------------------------------------
            # CURRENT RAIL POSITION
            # ------------------------------------------------
            slide_pose = magician.get_posel()['positionL']
            # ==================================================
            # ONLINE IMAGE JACOBIAN ESTIMATION
            # ==================================================
            if previous_x is not None:
                delta_x = (detected_x - previous_x)
                delta_l = (slide_pose - previous_l)
                # Avoid division by very small values
                if abs(delta_l) >= MIN_DELTA_L_FOR_JACOBIAN:
                    J_new = (delta_x / delta_l)
                    # ------------------------------------------------
                    # Reject obviously bad Jacobian estimates
                    # ------------------------------------------------
                    if (abs(J_new) >= MIN_JACOBIAN and abs(J_new) <= MAX_JACOBIAN):
                        # Preserve Jacobian sign
                        J_image = (JACOBIAN_ALPHA * J_new + (1 - JACOBIAN_ALPHA) * J_image)
                        print(f"Updated image Jacobian: " f"{J_image:.4f} px/mm")
            # Update previous measurements
            previous_x = detected_x
            previous_l = slide_pose
            # ==================================================
            # IMAGE-JACOBIAN IBVS CONTROL
            # ==================================================
            if abs(error_x) > DEADZONE:
                # ------------------------------------------------
                # IBVS CONTROL LAW
                #
                # delta_L =
                #
                # -lambda * inv(J) * error
                # ------------------------------------------------
                delta_l = (-LAMBDA * (error_x / J_image))
                # ------------------------------------------------
                # SAFETY LIMIT
                # ------------------------------------------------
                delta_l = max(-MAX_DELTA_L, min(delta_l, MAX_DELTA_L))
                # ------------------------------------------------
                # NEW RAIL POSITION
                # ------------------------------------------------
                current_l += delta_l
                # ------------------------------------------------
                # RAIL BOUNDARY PROTECTION
                # ------------------------------------------------
                current_l = max(RAIL_MIN, min(current_l,RAIL_MAX))           
            
            magician.set_ptpwithl_cmd(mode=1,  x=fixed_x, y=fixed_y,  z=fixed_z,  r=fixed_r,  l=current_l )
            #print( f"Error: {error_x:.2f} px | "  f"Gain: {KP_CURRENT:.3f} | "  f"Rail: {current_l:.2f} mm" )
            # 5. Output command to Dobot
              # We pass fixed arm coords while dynamically driving the rail 'l'
            if slide_pose>400 and slide_pose<450 and detections[0]['color']=='Red':
              magician.set_ptpwithl_cmd(mode=1,x=eject_x, y=fixed_y, z=fixed_z, r=fixed_r, l=slide_pose+50)
              running = False
            elif slide_pose>500 and slide_pose<550 and detections[0]['color']=='Green':
              magician.set_ptpwithl_cmd(mode=1,x=eject_x, y=fixed_y, z=fixed_z, r=fixed_r, l=slide_pose+50)
              running = False
            elif slide_pose>600 and slide_pose<650 and detections[0]['color']=='Blue':
              magician.set_ptpwithl_cmd(mode=1,x=eject_x, y=fixed_y, z=fixed_z, r=fixed_r, l=slide_pose+50)
              running = False

            # --- DO YOUR MAIN STAGE LOGIC HERE ---
            # This runs completely parallel without slowing down the frame grabber
            print(f"\n[Main Thread] Received {len(detections)} objects:")
            for item in detections:
              print(f" -> {item['color']} found at ({item['x']}, {item['y']})")
            
            # Signal queue that processing is done
            data_queue.task_done()
            
        
        except queue.Empty:
            # The queue was empty; yield CPU execution briefly or do other main tasks
            time.sleep(0.001) 
            
except KeyboardInterrupt:
    print("\nShutting down threads safely...")
    running = False
    detector_thread.join()


