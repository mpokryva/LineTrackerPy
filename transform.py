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
camera.resolution = (320, 240)
camera.framerate = 15
camera.hflip = True
camera.vflip = True
rawCapture = PiRGBArray(camera, size=camera.resolution)

time.sleep(0.1)

def getHoughLines(input):
    gray = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _,thresh = cv2.threshold(gray, 0, 255, \
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    _,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, \
            cv2.CHAIN_APPROX_SIMPLE)
    edges = cv2.Canny(thresh, 50, 150)
    minLineLen = 150
    maxLineGap = 10
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 5, minLineLen, maxLineGap)
    #lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50)
    return lines

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
    image = image[160:240]
    #image = image[300: 380]
    image = close(image)
    lines = getHoughLines(image)
    if lines is not None:
        for x1, y1, x2, y2 in lines[0]:
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.imshow("Frame", image)
    
    key = cv2.waitKey(1) & 0xFF

    # Clear the stream for the next frame.
    rawCapture.truncate(0)
    if key == ord("q"):
        break