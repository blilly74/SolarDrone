import numpy as np
import cv2 as cv

def processImage(frame, rectThreshold=0.85):  # <-- Added threshold parameter

    rectangularityScore = None
    originCoordinates = None
    approxCorners = None

    # --- Preprocessing Steps ---
    imgResize = cv.resize(frame, (640, 480), interpolation=cv.INTER_AREA)
    imgBlur = cv.bilateralFilter(imgResize, 9, 75, 75)
    imgHSV = cv.cvtColor(imgBlur, cv.COLOR_BGR2HSV)

    # --- Create Color Mask ---
    lowerBlue = np.array([100, 40, 40])
    upperBlue = np.array([130, 255, 255])
    lowerBlack = np.array([0, 0, 0])
    upperBlack = np.array([180, 255, 60])

    maskBlue = cv.inRange(imgHSV, lowerBlue, upperBlue)
    maskBlack = cv.inRange(imgHSV, lowerBlack, upperBlack)
    combinedMask = cv.bitwise_or(maskBlue, maskBlack)

    kernel7x7 = np.ones((7, 7), np.uint8)
    iterations = 2
    combinedMaskOpen = cv.morphologyEx(combinedMask, cv.MORPH_CLOSE, kernel7x7, iterations)
    combinedMaskClose = cv.morphologyEx(combinedMaskOpen, cv.MORPH_OPEN, kernel7x7, iterations)

    contours, _ = cv.findContours(combinedMaskClose, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if contours:
        largestContour = max(contours, key=cv.contourArea)
        contourArea = cv.contourArea(largestContour)

        if contourArea > 5000:

            rotatedRect = cv.minAreaRect(largestContour)
            (w, h) = rotatedRect[1]
            rectArea = w * h

            if rectArea > 0:
                rectangularityScore = contourArea / rectArea

                # --- Threshold Check Added Here ---
                if rectangularityScore >= rectThreshold:

                    cv.drawContours(imgResize, [largestContour], -1, (0, 255, 0), 2)

                    epsilon = 0.02 * cv.arcLength(largestContour, True)
                    approxCorners = cv.approxPolyDP(largestContour, epsilon, True)

                    if approxCorners is not None and len(approxCorners) == 4:
                        box = approxCorners.reshape(4, 2)

                        sums = box.sum(axis=1)
                        topLeftIndex = int(np.argmin(sums))
                        originCoordinates = box[topLeftIndex]

                        cv.drawContours(imgResize, [box], 0, (255, 0, 0), 2)
                        cv.circle(imgResize, tuple(originCoordinates), 10, (0, 0, 255), -1)

                else:
                    # Below threshold → treat as invalid panel
                    largestContour = None
                    originCoordinates = None
                    approxCorners = None

        else:
            largestContour = None
            rectangularityScore = None
    else:
        print("No contours found.")
        largestContour = None
        rectangularityScore = None

    return imgResize, largestContour, rectangularityScore, originCoordinates, approxCorners
