from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import cv2
import numpy as np
import pid
import sys
#sys.path.insert(0, "~/Camera/MotorHatLibrary/examples")
#sys.path.append("/MotorHatLibrary/examples")
from MotorHatLibrary.examples import Robot

camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 15
camera.hflip = True
camera.vflip = True
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

def getLargestContour(input):
    gray = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _,thresh = cv2.threshold(gray, 0, 255, \
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    _,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, \
            cv2.CHAIN_APPROX_SIMPLE)
    # Get largest contour (by area)
    maxIndex = 0
    maxArea = 0
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if area > maxArea:
            maxArea = area
            maxIndex = i
    #return max(contours, key = cv2.contourArea)
    return contours[maxIndex]

def drawContour(img, contour, color, thickness=8):
    cv2.drawContours(img, [contour], -1, color, thickness)

def close(img):
    kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

tau_p = 0.3
tau_d = 1.0
tau_i = 0.00
controller = pid.PIDController(tau_p, tau_i, tau_d)
robot = Robot.Robot()
move = True
for frame in camera.capture_continuous(rawCapture, format="bgr", \
        use_video_port=True):
    image = frame.array
    image = image[400:480]
    #image = image[300: 380]
    image = close(image)
    contour = getLargestContour(image)
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
    print("Angle: " + str(angle))
    # + angle = left, - angle = right
    # Draw stuff
    cv2.circle(image, (cx, cy), 4, (255, 255, 0), 2)
    drawContour(image, contour, (0, 0, 255), 4)
    cv2.line(image, (cx, cy), bottomCenter, (0, 255, 0), 4)
    
    steer = controller.pid(xDist, yDist, 1/camera.framerate)
    print("Steer: " + str(steer))
    
    cv2.imshow("Frame", image)
    
    speed = 150
    duration = 1/camera.framerate
    if (move and abs(xDist) > 10):
        if (xDist > 0):
            print("TURNING RIGHT")
            robot.right_deg(speed, duration)
        elif (xDist < 0):
            print("TURNING LEFT")
            robot.left_deg(speed, 5)
    if(move):
        robot.forward(speed, duration)

    key = cv2.waitKey(1) & 0xFF

    # Clear the stream for the next frame.
    rawCapture.truncate(0)
    if key == ord("q"):
        break
