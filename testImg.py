# NOTE: This script is for testing out without picamera installed

import cv2
import numpy as np

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

test = cv2.imread('black_strip.jpg')
test = cv2.resize(test, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)

#testGray = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY)
test = close(test)
contour = getLargestContour(test, 50, False)

moment = cv2.moments(contour)
cx = int(moment["m10"]/moment["m00"])
cy = int(moment["m01"]/moment["m00"])
width = np.size(test, 1)
height = np.size(test, 0)
bottomCenter = (width/2, height)

testGray = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY)
drawContour(test, contour, (0, 255, 0), 2)
cv2.line(test, (cx, cy), bottomCenter, (0, 255, 0), 4)
cv2.imshow('result', test)

if cv2.waitKey(0) == ord('q'):
	cv2.destroyAllWindows
