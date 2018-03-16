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
camera.framerate = 40
camera.hflip = True
camera.vflip = True
rawCapture = PiRGBArray(camera, size=camera.resolution)

time.sleep(0.1)

def getLargestContour(input, threshold, otsu=True):
    gray = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    threshType = cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU if (otsu) \
            else cv2.THRESH_BINARY_INV
    _,thresh = cv2.threshold(gray, threshold, 255, \
            threshType)
    _,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, \
            cv2.CHAIN_APPROX_SIMPLE)
    # Get largest contour (by area)
    maxIndex = 0
    maxArea = 0
    if (len(contours) > 0):
        return max(contours, key = cv2.contourArea)
    else:
        return None


def drawContour(img, contour, color, thickness=8):
    cv2.drawContours(img, [contour], -1, color, thickness)

def close(img):
    kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

turn_p = 0.5
turn_d = 0.01
turn_i = 0.005
turn_s = 0.005
maxSpeed = 255.0
turnControl = pid.PIDController(turn_p, turn_i, turn_d, turn_s)
speedControl = pid.PIDController(0, 0, 0, turn_s)
robot = Robot.Robot()
move = True
i = 0
max_tries = 5
current_try = 1
prev_speed = maxSpeed
prev_steer = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", \
        use_video_port=True):
    image = frame.array
    image = image[camera.resolution[1] - 80: camera.resolution[1]]
    image = close(image)
    threshold = 50
    contour = getLargestContour(image, threshold, False) # Don't do Otsu.
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
        cx = int(moment["m10"]/moment["m00"])
        cy = int(moment["m01"]/moment["m00"])
    else:
        cx, cy = 0, 0
    # Calculate angle.
    width = np.size(image, 1)
    height = np.size(image, 0)
    bottomCenter = (width/2, height)
    xDist = (cx - bottomCenter[0])
    yDist = abs(cy - bottomCenter[1])
    angle = np.arctan2(xDist, yDist) * (180 / np.pi)
    #print("Angle: " + str(angle))
    # + angle = left, - angle = right
    # Draw stuff
    cv2.circle(image, (cx, cy), 4, (255, 255, 0), 2)
    drawContour(image, contour, (0, 0, 255), 4)
    cv2.line(image, (cx, cy), bottomCenter, (0, 255, 0), 4)
    
    MAX_ABS_SPEED = 255.0
    turn_p = maxSpeed / MAX_ABS_SPEED#0.5
    turn_d = maxSpeed / (MAX_ABS_SPEED * 50)#0.01
    turn_i = maxSpeed / (MAX_ABS_SPEED * 100)#0.005
    turn_s = maxSpeed / (MAX_ABS_SPEED * 25)
    turnControl.setGains(turn_p, turn_i, turn_d, 0)
    speedControl.setGains(0, 0, 0, turn_s)
    steer = turnControl.pid(xDist, yDist, 1/camera.framerate)
    speedDiff = speedControl.pid(xDist, yDist, 1/camera.framerate)
    #print("Steer: " + str(steer))
    #print("SpeedDiff: " + str(speedDiff)) 
    cv2.imshow("Frame", image)
    
    speed = maxSpeed + speedDiff
    prev_speed = speed
    prev_steer = steer
    duration = 1/camera.framerate
    errorThresh = 5.0
    if (move and abs(xDist) > errorThresh):
        absSteer = abs(steer)
        robot.smooth_turn(int(speed), int(steer))
    key = cv2.waitKey(1) & 0xFF

    # Clear the stream for the next frame.
    rawCapture.truncate(0)
    if key == ord("q"):
        break
    elif key == ord("m"):
        move = not move
        if not move:
            robot.stop()
    i += 1
