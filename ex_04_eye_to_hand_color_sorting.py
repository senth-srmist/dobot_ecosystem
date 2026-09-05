import sys
sys.path.append(r'C:\SENTH\VISION')
from helper_04 import HikrobotCamera
from helper_04 import read_calib_file
from helper_04 import move_and_wait
import math
import cv2
import numpy as np
import time
from dobotapi import Dobot
from matplotlib import pyplot as plt

#%%
robot=Dobot('COM4')
robot.connect()
#% ALL LIGHTING IN LAB ON, EXPOSURE: 20000.00, GAIN: 10.0 in MVS SOFTWARE
cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera", 640, 480)
# ---------- SORTING ----------
cap = HikrobotCamera()

# frame must be RGB
while True:   
    move_and_wait(robot, 0, -250, -20) 
    loop_count = 1
    object_coordinates = []
    frame = cap.read() 
    frame_swap = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame_swap,cv2.COLOR_RGB2GRAY)
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


        center = (int(u), int(v))
        print(center)
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
        print('Color is:', RGB_list[max_index], "Center:", center, "R:", round(mean_R), "G:", round(mean_G), "B:", round(mean_B))           
        
        text = str(mean_R)+','+str(mean_G)+','+str(mean_B)
        position =  (center[0]-40,center[1]) # X=50, Y=100
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 2
        color = (0, 0, 0)  # Green in BGR
        thickness = 2
        
# Draw the text on the image
        cv2.putText(frame_swap, color_id+':'+str(loop_count), position, font, scale, color, thickness, cv2.LINE_AA)
        cv2.circle(frame_swap, center, 50, (0, 0, 0), thickness=10, lineType=None, shift=None)
        cv2.imshow("Camera", frame_swap)
        loop_count+=1
        # Pixel -> Dobot X,Y
        p = np.array(
            [[[center[0], center[1]]]],
            dtype=np.float32
        )

        predicted = cv2.perspectiveTransform(np.array([[center[0], center[1]]], dtype=np.float32).reshape(-1, 1, 2), H).reshape(-1, 2)
        x = float(predicted[0][0])
        y = float(predicted[0][1])  
        move_and_wait(robot, x, y, -20)
        move_and_wait(robot, x, y, -47)
        robot.suction_cup.suck()
        time.sleep(0.3)   
        move_and_wait(robot, x, y, -20)
        move_and_wait(robot, 0, -250, -20)   
        robot.suction_cup.idle()
        break
        #object_coordinates.append((x,y,color_id))
        
    
robot.close()    
cap.close()
cv2.destroyAllWindows()

#%% CALIBRATION WITHIN OPENCV
# Put a calibration object at each point.
# Press C to pause he acquisition
# Click on the screen and press space bar
# Enter its Dobot X,Y coordinates when asked.
cap = HikrobotCamera()

cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Calibration", 2592, 1944)
robot_points = []
image_points = []

print("CALIBRATION")
print("Click the same physical point visible in the camera.")
print("Then enter its Dobot X and Y coordinates.")

for i in range(9):
    while True:
        frame = cap.read()
        cv2.imshow("Calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
   

    # Get clicked pixel
    clicked = cv2.selectROI("Calibration", frame, False)
    u = clicked[0] + clicked[2] / 2
    v = clicked[1] + clicked[3] / 2
    
    print("Point", i + 1)
    print("Enter Dobot X:")
    x = float(input())
    print("Enter Dobot Y:")
    y = float(input())

    image_points.append([u, v])
    robot_points.append([x, y])

image_points = np.float32(image_points)
robot_points = np.float32(robot_points)
cap.close()
cv2.destroyAllWindows()
#%% CALIBRATION - COLLECTION OF CORRESPONDING POINTS
image_points = np.array([
    [753, 1192],
    [752, 920],
    [752, 641],
    [1027, 1192],
    [1027, 920],
    [1027, 643],
    [1302, 1192],
    [1300, 917],
    [1300, 641]
], dtype=np.float32)

robot_points = np.array([
    [274.87,  19.78],
    [274.87,  -5.18],
    [275.24, -30.88],
    [249.63,  20.05],
    [249.63,  -5.49],
    [249.89, -29.85],
    [224.48,  19.61],
    [224.48,  -4.89],
    [224.84, -30.69]
], dtype=np.float32)

# Image pixel -> robot XY
H, mask = cv2.findHomography(
    image_points,
    robot_points,
    method=0
)

print("Homography:")
print(H)

#image_points,robot_points = read_calib_file(filename = r'C:\SENTH\VISION\calib_data_obj_top.txt')
#%% CALIBRATION - GET WORLD POINTS FROM IMAGE POINTS

predicted = cv2.perspectiveTransform(
    np.array(image_points, dtype=np.float32).reshape(-1, 1, 2),
    H
).reshape(-1, 2)

for i in range(len(robot_points)):
    error = np.linalg.norm(predicted[i] - robot_points[i])

    print(
        f"Point {i+1}: "
        f"actual={robot_points[i]}, "
        f"predicted={predicted[i]}, "
        f"error={error:.2f} mm"
    )

print("Mean error:",
      np.mean(np.linalg.norm(
          predicted - np.array(robot_points),
          axis=1
      )),
      "mm")
#%%




