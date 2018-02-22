from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


time.sleep(0.1)

def getLargestContour(input):
    gray = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _,thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    #ratio = 3
    #lowThresh = 100
    #thresh = cv2.Canny(thresh, lowThresh, lowThresh * ratio)
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

image = cv2.imread("test.jpg")
image = image[400:480]
image = close(image)
contour = getLargestContour(image)
moment = cv2.moments(contour)
if (moment["m00"] != 0):
    cx = int(moment["m10"]/moment["m00"])
    cy = int(moment["m01"]/moment["m00"])
else:
    cx, cy = 0, 0
# Calculate angle.
width = np.size(image, 0)
height = np.size(image, 1)
xDist = abs(cx - width/2)
yDist = abs(cy - height)
angle = np.arctan2(xDist, yDist) * (180 / np.pi)
print("Angle: " + str(angle))
# + angle = left, - angle = right

# Draw stuff
cv2.circle(image, (cx, cy), 4, (255, 255, 0), 2)
drawContour(image, contour, (0, 0, 255), 10)
#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#gray = cv2.GaussianBlur(gray, (3, 3), 0)
#_,thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Image", 400, 400)
cv2.imshow("Image", image)

key = cv2.waitKey(0)

