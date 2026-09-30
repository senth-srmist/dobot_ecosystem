# version: Python3
import sys
sys.path.append(r'C:\Users\Admin\Desktop')
from DobotEDU import *
magician.set_converyor(index=magician.Stepper1,enable=True,speed=-10.0)
#sys.path.append(r'D:\LECTURE\0SEMESTER_WISE\ODD 2026-27\21MHE457L - Robot Programming\Codes\Helper_Files')
from helper_04 import HikrobotCamera
import cv2
import numpy as np
import time
import statistics

#%%
H = np.array([[-8.03640003e-02, 2.60845155e-03, 3.38878281e+02],
[ 1.96958348e-03, 8.09136248e-02, -2.73550431e+02],
[-1.11548214e-05, -1.23961313e-05, 1.00000000e+00]],dtype=np.float32)

#% ALL LIGHTING IN LAB ON, EXPOSURE: 20000.00, GAIN: 10.0 in MVS SOFTWARE
cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
# ---------- SORTING ----------
cap = HikrobotCamera()
threshold = 50
# frame must be RGB

previous_y = 0
previous_time = 0
atleast_one_object_tracked = False
loop_count = 0
object_left_fov = False
small_object = False
velocity = []
object_coordinates = []
loop_count = 1


while True:   
 
    
    frame = cap.read() 
    frame = frame[:,512:1600,:]
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
    frame_swap = cv2.cvtColor(cleaned_frame,cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame_swap,cv2.COLOR_RGB2GRAY)
    #hist = cv2.calcHist([gray],[0],None, [256], [0,256])
    thresh, bw = cv2.threshold(gray, 50, 255, type=cv2.THRESH_BINARY)   
   

    contours, _ = cv2.findContours(
            bw,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
    
   
    current_time = time.time()
    print('KEY DATA', atleast_one_object_tracked, loop_count, small_object)
    
    if atleast_one_object_tracked == True and small_object==True:
        break
    
    for c in contours:
        area = cv2.contourArea(c)
        if area < 90000:
            small_object=True
            continue
        else: 
            small_object=False
        loop_count+=1
        if loop_count>2:
            atleast_one_object_tracked = True
            
        rect = cv2.minAreaRect(c)
    
        (cx, cy), (w, h), angle = rect
    
        # Convert OpenCV's angle into the orientation of the long side
        if w < h:
            orientation = angle
        else:
            orientation = angle + 90
    
        orientation %= 180
    
        #print(f"Center: ({cx:.1f}, {cy:.1f})")
        #print(f"Width: {w:.1f}, Height: {h:.1f}")
        #print(f"Orientation: {orientation:.1f} degrees")
    
        # Draw the rotated rectangle
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(frame_swap, [box], 0, (0, 255, 0), 2)
    
        # Draw center
        cv2.circle(frame_swap, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue

        u = M["m10"] / M["m00"]
        v = M["m01"] / M["m00"]
        

        center = (int(u), int(v))
        #print(center)
        # All pixels inside and on the contour boundary
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [c], -1, 255, cv2.FILLED)
    
        pixels = frame[mask == 255]
    
        mean_R, mean_G, mean_B = np.mean(pixels, axis=0)
        mean_list = [mean_R, mean_G, mean_B]
        max_index = mean_list.index(max(mean_list))
        RGB_list = ['RED','GRN','BLU',"YLO"]
        global_mean = (mean_R+mean_G+mean_B)/3
        if global_mean>160:
            color_id=RGB_list[3]
        else:
            color_id=RGB_list[max_index]
        #print('Color is:', RGB_list[max_index], "Center:", center, "R:", round(mean_R), "G:", round(mean_G), "B:", round(mean_B))           
        
        text = str(mean_R)+','+str(mean_G)+','+str(mean_B)
        position =  (center[0]-40,center[1]) # X=50, Y=100
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 2
        color = (0, 0, 0)  # Green in BGR
        thickness = 2
        
# Draw the text on the image
        cv2.putText(frame_swap, color_id+':'+str(loop_count), position, font, scale, color, thickness, cv2.LINE_AA)
        cv2.circle(frame_swap, center, 50, (0, 0, 0), thickness=10, lineType=None, shift=None)
        
        loop_count+=1
        # Pixel -> Dobot X,Y
        p = np.array(
            [[[center[0], center[1]]]],
            dtype=np.float32
        )

        predicted = cv2.perspectiveTransform(np.array([[center[0], center[1]]], dtype=np.float32).reshape(-1, 1, 2), H).reshape(-1, 2)
        x = float(predicted[0][0])
        y = float(predicted[0][1])  
        time.sleep(0.3)   
        
        dt = current_time - previous_time
        dy = y - previous_y

        per_frame_velocity = dy / dt
        velocity.append(per_frame_velocity)
        object_coordinates.append((x,y,color_id))
        print(
                f"Y = {y:.1f}, "
                f"Velocity = {per_frame_velocity:.1f} mm/s"
            )
        
        
        
        previous_y = y
        previous_time = current_time
    
    cv2.imshow("Camera", frame_swap)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
print("LOOP EXITED")       
start_time = time.time()
# Calculate the median
median_velocity = statistics.median(velocity)
#mean_velocity = sum(velocity)/len(velocity)
final_y = object_coordinates[-1][1]
required_waiting_time = abs(final_y/median_velocity)
elasped_time = 0
while elasped_time<=required_waiting_time:
    elasped_time = time.time()-start_time
print('DONNEEEE') 
magician.set_converyor(index=magician.Stepper1,enable=False,speed=0)
    
cap.close()
cv2.destroyAllWindows()































