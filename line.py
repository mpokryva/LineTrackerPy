from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import cv2
import numpy as np
import pid
import sys
from MotorHatLibrary.examples import Robot

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 15
camera.hflip = True
camera.vflip = True
rawCapture = PiRGBArray(camera, size=camera.resolution)

time.sleep(0.1)

# Gets the largest contour in the image, by area,
# as a list of (x,y) coordinate pairs.
def getLargestContour(input, threshold, otsu=True):
    # Convert the image to grayscale, 
    gray = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur to decrease noise.
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    '''
    Threshold the image, converting the irrelevant parts to 0 (black)
    and the relevant parts (the line) to 255 (white). Notice the use of 
    cv2.THRESH_BINARY_INV. This tells OpenCV to convert things BELOW 
    the threshold (as opposed to above) to white. This is important 
    since our line is black. Otsu thresholding is an adaptive thresholding 
    algorithm, which I found was unnecessary for this project.
    '''
    threshType = cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU if (otsu) \
            else cv2.THRESH_BINARY_INV
    _,thresh = cv2.threshold(gray, threshold, 255, \
            threshType)
    thresh = close(thresh)
    # Find all contours in the thresholded image.
    _,contours,_ = cv2.findContours(thresh, cv2.RETR_TREE, \
            cv2.CHAIN_APPROX_SIMPLE)
    # Return the largest contour (by area).
    if (len(contours) > 0):
        return max(contours, key = cv2.contourArea)
    else:
        return None

# Draw the contours on the original frame.
def drawContour(img, contour, color, thickness=8):
    cv2.drawContours(img, [contour], -1, color, thickness)

# Morphologically close the image (remove gaps).
def close(img):
    kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

# Initialize PID controllers and robot.
maxSpeed = 255.0
MAX_ABS_SPEED = 255.0
turn_p = maxSpeed / MAX_ABS_SPEED
turn_d = maxSpeed / (MAX_ABS_SPEED * 500)
turn_i = maxSpeed / (MAX_ABS_SPEED * 500)
turn_s = maxSpeed / (MAX_ABS_SPEED * 25)
turnControl = pid.PIDController(turn_p, turn_i, turn_d, 0)
speedControl = pid.PIDController(0, 0, 0, turn_s)
robot = Robot.Robot()
move = True
i = 0
# Maximum number of frames without an identified line before robot stops.
max_tries = 5
current_try = 1
prev_speed = maxSpeed
prev_steer = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", \
        use_video_port=True):
    image = frame.array
    # We crop the image vertically, to eliminate irrelevant details.
    image = image[camera.resolution[1] - 80: camera.resolution[1]]
    # Any pixel with a value above this threshold will be set to 0 (black).
    threshold = 50
     # Don't do Otsu (I found it performs better without).
    contour = getLargestContour(image, threshold, False)
    '''
    Optimization for handling slight overshoot of turns, when line
    briefly goes out of view.
    '''
    if (contour is None):
        if (current_try > max_tries):
            print("No lines found... Stopping.")
            break
        print("No lines found... Try #" + str(current_try))
        current_try += 1
        robot.smooth_turn(int(prev_speed), int(prev_steer))
        rawCapture.truncate(0)
        continue
    moment = cv2.moments(contour)
    if (moment["m00"] != 0):
         # Get the center x.
        cx = int(moment["m10"]/moment["m00"])
         # Get the center y.
        cy = int(moment["m01"]/moment["m00"])
    else:
        cx, cy = 0, 0
    
    width = np.size(image, 1)
    height = np.size(image, 0)
    bottomCenter = (width/2, height)
    xDist = (cx - bottomCenter[0])
    yDist = abs(cy - bottomCenter[1])
    # Calculate angle.
    #angle = np.arctan2(xDist, yDist) * (180 / np.pi)

    # Draw stuff
    cv2.circle(image, (cx, cy), 4, (255, 255, 0), 2)
    drawContour(image, contour, (0, 0, 255), 4)
    cv2.line(image, (cx, cy), bottomCenter, (0, 255, 0), 4)
    
    # Get steer and speed corrections from PID controllers.
    steer = turnControl.pid(xDist, yDist, 1/camera.framerate)
    speedDiff = speedControl.pid(xDist, yDist, 1/camera.framerate)

    # Display the current frame.
    cv2.imshow("Frame", image)
    
    speed = maxSpeed + speedDiff
    prev_speed = speed
    prev_steer = steer
    errorThresh = 5.0
    if (move and abs(xDist) > errorThresh):
        absSteer = abs(steer)
        robot.smooth_turn(int(speed), int(steer))
    key = cv2.waitKey(1) & 0xFF

    # Clear the stream for the next frame.
    rawCapture.truncate(0)
    '''
    If "q" is pressed, stop. 
    If "m" is pressed, toggle robot movement.
    This is useful for debugging, since you can pick the robot up,
    move it somewhere else, and then press "m" again.
    '''
    if key == ord("q"):
        break
    elif key == ord("m"):
        move = not move
        if not move:
            robot.stop()
    i += 1
