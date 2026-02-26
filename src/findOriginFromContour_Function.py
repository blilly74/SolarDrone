import cv2 as cv
import numpy as np

def findOriginFromContour(contour):
    """
    Takes a contour and find the top-left origin coordinate.
    """
    # Approximate contour to polygon
    epsilon = 0.03 * cv.arcLength(contour, True)
    approxCorners = cv.approxPolyDP(contour, epsilon, True)

    if len(approxCorners) ==4:
        # Reshape to (4, 2)
        box = approxCorners.reshape(4, 2)
        # Find the top-left corner (smallest x + y sum)
        sums = box.sum(axis=1)
        topLeftIndex = np.argmin(sums)
        originCoordinates = box[topLeftIndex]
        return originCoordinates
    else:
        return None