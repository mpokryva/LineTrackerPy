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
camera.framerate = 30
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

turn_p = 0.5#0.1
turn_d = 0.01#0.01
turn_i = 0.005#0.005
maxSpeed = 100.0
forwardSpeed = maxSpeed
turnControl = pid.PIDController(turn_p, turn_i, turn_d)
forwardControl = pid.PIDController(0, 0, 0) # Gains set later.
robot = Robot.Robot()
move = True
i = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", \
        use_video_port=True):
    image = frame.array
    image = image[camera.resolution[1] - 80: camera.resolution[1]]
    #image = image[300: 380]
    image = close(image)
    threshold = 100
    contour = getLargestContour(image, threshold, False) # Don't do Otsu.
    if (contour is None):
        print("No lines found... Stopping.")
        break
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
    #print("xDist: " + str(xDist))
    #print("yDist: " + str(yDist))
    angle = np.arctan2(xDist, yDist) * (180 / np.pi)
    #print("Angle: " + str(angle))
    # + angle = left, - angle = right
    # Draw stuff
    cv2.circle(image, (cx, cy), 4, (255, 255, 0), 2)
    drawContour(image, contour, (0, 0, 255), 4)
    cv2.line(image, (cx, cy), bottomCenter, (0, 255, 0), 4)
    
    steer = turnControl.pid(xDist, yDist, 1/camera.framerate)
    print("Steer: " + str(steer))
    
    cv2.imshow("Frame", image)
    
    MAX_ABS_SPEED = 255.0
    speed = maxSpeed
    duration = 1/camera.framerate
    errorThresh = 5.0
    if (move and abs(xDist) > errorThresh):
        absSteer = abs(steer)
        turnSpeed = min(max(speed * absSteer, 0), MAX_ABS_SPEED)
        turnSpeed = int(turnSpeed)
        #print("turnSpeed: " + str(turnSpeed))
        #if (xDist > 0):
            #print("TURNING RIGHT")
            #robot.right_deg(turnSpeed, duration)
        #elif (xDist < 0):
            #print("TURNING LEFT")
            #robot.left_deg(turnSpeed, duration)
        robot.smooth_turn(int(speed), int(steer))
    if(move):
        forward_p = (maxSpeed) / (MAX_ABS_SPEED)
        forward_d = (maxSpeed) / (MAX_ABS_SPEED * 10)
        forward_i = (maxSpeed) / (MAX_ABS_SPEED * 100)
        forwardControl.setGains(forward_p, forward_i, forward_d)
        forwardSteer = forwardControl.pid(xDist, yDist, 1/camera.framerate)
        forwardSpeed = speed - abs(forwardSteer)
        forwardSpeed = min(max(0, forwardSpeed), MAX_ABS_SPEED)
        #robot.forward(int(forwardSpeed))
        #print("forwardSpeed: " + str(forwardSpeed))
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
