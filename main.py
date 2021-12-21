import cv2
import numpy as np
import math
import sys
import time

# Setting Video Parameters and Opening Video
fps = 30.0
frameW = 640
frameH = 480
PATH = r"C:\Users\asafw\Documents\RoboDeck\Deck Videos\Deck Video7.mp4"
PATH2 = R"C:\Users\asafw\Documents\RoboDeck\Deck Videos\DeckVideo2.mp4"

PATH_ORIG = r"Resources/Captured Videos Pi Cam/Deck Video Raw1.avi"
cap = cv2.VideoCapture(PATH2)
if not cap.isOpened():
    print("Cannot Open Camera")
    sys.exit()
cap.set(3, frameW)
cap.set(4, frameH)
cap.set(5, fps)

# ROI Top and Bottom
yTop = 0
yBottom = 170

yTopLarge = 310
yBottomLarge = 480

# ROI for searching and for tracking
roi0 = np.array([[[75, yBottom], [280, yTop], [360, yTop], [565, yBottom]]], np.int32)
roi0Draw = np.array([[[75, yBottomLarge], [280, yTopLarge], [360, yTopLarge], [565, yBottomLarge]]], np.int32)

# Parameters for Canny Edge Detection and Hough Line
apertureSize = 5
cannyMinVal = 50
cannyMaxVal = 300
houghThreshold = 50

robotAxis = [(320, 480), (320, 0)]
vanishPointY = 242

def main():
    tracking = False
    trackingCounter = 0
    lostCounter = 0
    runTimeLog = np.array([])
    frameCounter = 0
    xStartPrev = 0
    xEndPrev = 0
    time.sleep(1)

    while True:
        tStart = time.perf_counter()
        success, imgRaw = cap.read()
        if imgRaw is None:
            print("No Image Read")
            sys.exit()

        imgResult = np.copy(imgRaw)
        imgRaw = imgRaw[310:480, 0:640]
        cv2.rectangle(imgResult, (0, 0), (220, 95), (100, 100, 100), -1)

        # Choose ROI and Create Mask
        mask = np.zeros_like(imgRaw)
        if not tracking:
            cv2.fillConvexPoly(mask, roi0, (255,255,255))
            imgMasked = cv2.bitwise_and(imgRaw, mask)
            cv2.polylines(imgResult, roi0Draw, True, (255, 255, 255), 2)
        else:
            roi1, roi1Draw = boundingBox(xStartPrev, xEndPrev)
            cv2.fillConvexPoly(mask, roi1, (255,255,255))
            imgMasked = cv2.bitwise_and(imgRaw, mask)
            cv2.polylines(imgResult, roi1Draw, True, (255, 255, 255), 2)

        # Convert Image to HSV
        imgHSV = cv2.cvtColor(imgMasked, cv2.COLOR_BGR2HSV)

        # Range Limiting Mask
        gapMask = hsvFilter(imgHSV)
        imgBWise = cv2.bitwise_and(imgMasked, imgMasked, mask=gapMask)
        imgBlurred = cv2.GaussianBlur(imgBWise, (7, 7), 1)

        # Canny Edge Detection
        imgEdges = cv2.Canny(imgBlurred, cannyMinVal, cannyMaxVal, apertureSize=apertureSize)

        # Hough Line Detection
        startXs, endXs = findLines(imgEdges)


        # asaf's test
        startXsP, endXsP, startYsP, endYsP = findLinesP(imgEdges)
        print("test test test test test test test test test test :")
        print(f"startXsP : {startXsP}")
        print("test test test test test test test test test test :")
        print(f"startXs : {startXs}")
        print("test test test test test test test test test test :")




        # asaf's test


        # Filtering Detected Lines
        if startXs:
            startFiltered, endFiltered, gapStart, gapEnd = removeOutliers(startXs, endXs)

            # Drawing Filtered Lines and Main Line
            # if isinstance(startFiltered, np.ndarray):
                # for (xA, xB) in zip(startFiltered, endFiltered):
                    # cv2.line(imgResult, (xA, yBottomLarge), (xB, yTopLarge), (100, 255, 100), 2)
            cv2.line(imgResult, (gapStart, yBottomLarge), (gapEnd, yTopLarge), (255, 100, 255), 3) #(255, 100, 255)
            cv2.line(imgResult, (gapStart, yBottomLarge), (gapEnd, yTopLarge), (22, 100, 43), 3) #(255, 100, 255)


            # asaf's test
            #cv2.line(imgResult, (startXsP[0], yBottomLarge), (endXsP[0], yTopLarge), (22, 100, 43), 3) #(255, 100, 255)
            # asaf's test

            print("Gap Start: " + str(gapStart))
            print("Gap End: " + str(gapEnd))
            # Enter Tracking Mode Once Gap Is Found
            gapFound = True
            # asaf's changes
            lostCounter = 0
            # asaf's changes
            frameDif = abs(xStartPrev - gapStart)
            print(frameDif)
            if frameDif < 20:
                trackingCounter += 1
            else:
                trackingCounter = 0
            if trackingCounter == 5:
                tracking = True
                trackingCounter = 0
            xStartPrev = gapStart
            xEndPrev = gapEnd

        # Gap Not Found
        else:
            gapFound = False
            print("No Gap Found")
            lostCounter += 1
            if lostCounter == 5:
                tracking = False
                lostCounter = 0

        # Calculate Robot Offset and Angle
        if gapFound:
            gapStartPoint = np.asarray((gapStart, yBottomLarge), dtype=np.int32)
            gapEndPoint = np.asarray((gapEnd, yTopLarge), dtype=np.int32)
            if robotAxis[0][0] == gapEnd and robotAxis[0][0] == gapStart:
                gapAngle = 0.0
            else:
                vanishPointX = int(((yTopLarge-vanishPointY)*(gapEnd-gapStart)/(yBottomLarge-yTopLarge)) + gapEnd)
                perspectiveLine = [robotAxis[0], (vanishPointX, vanishPointY)]
                cv2.line(imgResult, perspectiveLine[0], perspectiveLine[1], (255, 0, 0), 2)

                gapAngle = math.atan2((vanishPointY - yBottomLarge), (vanishPointX-robotAxis[0][0]))
                gapAngle = round(90 + math.degrees(gapAngle), 1)

            gapOffset = np.linalg.norm(np.cross((gapEndPoint-gapStartPoint), (gapStartPoint-robotAxis[0])))/np.linalg.norm(gapEndPoint-gapStartPoint)
            # gapOffset = (gapStart - robotAxis[0][0])

            cv2.putText(imgResult, "offset: " + str(int(gapOffset)) + " pixels", (0, 85), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)
            cv2.putText(imgResult, "angle: " + "{:.1f}".format(gapAngle), (0, 55), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)

        else:
            cv2.putText(imgResult, "No Gap Found", (0, 60), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1)

        tEnd = time.perf_counter()
        elapsedTime = tEnd - tStart
        runTimeLog = np.append(runTimeLog, elapsedTime)
        avgFPS = np.mean(runTimeLog[-50:])
        actualFPS = round(1/avgFPS)
        cv2.putText(imgResult, "fps: " + str(actualFPS), (0, 25), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)

        # imgResult = cv2.addWeighted(imgRaw, 0.35, imgResult, 0.65, 0.0)

        # Drawing helper lines
       #  cv2.line(imgResult, (gapEnd, yTopLarge), (gapStart, yTopLarge), (100, 100, 100), 2)
        # cv2.line(imgResult, (gapStart, yTopLarge), (gapStart, yBottomLarge), (100, 100, 100), 2)
        cv2.line(imgResult, (robotAxis[0][0], robotAxis[0][1]), (robotAxis[1][0], robotAxis[1][1]), (0, 100, 255), 2)
        cv2.line(imgResult, (robotAxis[0][0], robotAxis[0][1]), (robotAxis[1][0], robotAxis[1][1]), (0, 100, 255), 2)

        frameCounter += 1
        if frameCounter % 100 == 0:
            print("Actual FPS: " + str(actualFPS))

        # Show Images at Different Stages in the Algo
        # cv2.imshow("Img Original ROI", imgMasked)
        # cv2.imshow("Img HSV", imgHSV)
        # cv2.imshow("Img HSV Fixed", imgHSVFixed)
        cv2.imshow("Img Gap Mask", gapMask)
        # cv2.imshow("Img Masked", imgBWise)
        # cv2.imshow("Img Blurred", imgBlurred)
        cv2.imshow("Img Edges", imgEdges)
        cv2.imshow("Img Result", imgResult)
        cv2.waitKey(0)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


def hsvFilter(imgHSVFixed):
    # gapVal = -526.78 + (3.91 * abs(val1-val)) + (580.56 * val/val1)
    # valAdjustPos = 30.0
    # valAdjustNeg = 50.0

    hueMin, hueMax = 1, 179
    satMin, satMax = 0, 255
    valMin, valMax = 0, 60
    # valMin, valMax = (gapVal - valAdjustNeg), (gapVal + valAdjustPos)

    lower = np.array([hueMin, satMin, valMin])
    upper = np.array([hueMax, satMax, valMax])
    mask = cv2.inRange(imgHSVFixed, lower, upper)
    mask = cv2.dilate(mask, None, iterations=1)

    return mask

def boundingBox(xBottom, xTop):
    p1 = xTop - 30
    p2 = xBottom - 60
    p3 = xTop + 30
    p4 = xBottom + 60
    box = np.array([[[p3, yTop], [p1, yTop], [p2, yBottom], [p4, yBottom]]], np.int32)
    boxDraw = np.array([[[p3, yTopLarge], [p1, yTopLarge], [p2, yBottomLarge], [p4, yBottomLarge]]], np.int32)

    return box, boxDraw

def findLines(img):
    foundStartXs = []
    foundEndXs = []

    lines = cv2.HoughLines(img, 1, math.radians(1), houghThreshold)
    if lines is None:
        print("No Lines Found")
        return None, None

    for line in lines:
        for rho, theta in line:
            if theta <= math.radians(55) or theta >= math.radians(125):
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho

                # x = x0 - ((y - y0) / a) * b
                # y = y0 + ((x0 - x) / b) * a

                x1 = round(x0 - ((yBottom - y0) / a) * b)
                x2 = round(x0 - ((yTop - y0) / a) * b)

                foundStartXs.append(x1)
                foundEndXs.append(x2)

    return foundStartXs, foundEndXs


def findLinesP(img):
    foundStartXsP = []
    foundStartYsP = []

    foundEndXsP = []
    foundEndYsP = []


    linesP = cv2.HoughLinesP(img, 1, np.pi / 180, 50, None, 50, 10) # treshold 50
    if linesP is None:
        print("No Lines Found")
        return None, None

    for i in range(0, len(linesP)):
        l = linesP[i][0]
        x1 = l[0]
        y1 = l[1]
        x2 = l[2]
        y2 = l[3]
        foundStartXsP.append(x1)
        foundStartYsP.append(y1)
        foundEndXsP.append(x2)
        foundEndYsP.append(y2)

        img_withP = np.copy(img)

        cv2.line(img_withP, (l[0], l[1]), (l[2], l[3]), (255, 100, 255), 3, cv2.LINE_AA)
        cv2.imshow("img with P", img_withP)

    return foundStartXsP, foundEndXsP, foundStartYsP, foundEndYsP


def removeOutliersForP(lineS, lineE):
    lineStart = np.array(lineS)
    lineEnd = np.array(lineE)
    mainStart = lineStart
    mainEnd = lineEnd
    if lineStart.size == 1:
        return lineStart, lineEnd, int(mainStart), int(mainEnd)

    elif lineStart.size == 2:
        return np.mean(lineStart), np.mean(lineEnd), int(np.mean(mainStart)), int(np.mean(mainEnd))

    else:
        startStd = np.std(lineStart, axis=0)
        startMean = np.mean(lineStart, axis=0)
        cutOff = startStd * 1
        startLower = startMean - cutOff
        startUpper = startMean + cutOff
        i = 0
        deleteList = []
        for x in lineStart:
            if x < startLower or x > startUpper:
                deleteList.append(i)
            i += 1

        startFiltered = np.delete(lineStart, deleteList)
        endFiltered = np.delete(lineEnd, deleteList)
        mainStart = int(np.mean(startFiltered))
        mainEnd = int(np.mean(endFiltered))

        return startFiltered, endFiltered, mainStart, mainEnd



def removeOutliers(lineS, lineE):
    lineStart = np.array(lineS)
    lineEnd = np.array(lineE)
    mainStart = lineStart
    mainEnd = lineEnd
    if lineStart.size == 1:
        return lineStart, lineEnd, int(mainStart), int(mainEnd)

    elif lineStart.size == 2:
        return np.mean(lineStart), np.mean(lineEnd), int(np.mean(mainStart)), int(np.mean(mainEnd))

    else:
        startStd = np.std(lineStart, axis=0)
        startMean = np.mean(lineStart, axis=0)
        cutOff = startStd * 1
        startLower = startMean - cutOff
        startUpper = startMean + cutOff
        i = 0
        deleteList = []
        for x in lineStart:
            if x < startLower or x > startUpper:
                deleteList.append(i)
            i += 1

        startFiltered = np.delete(lineStart, deleteList)
        endFiltered = np.delete(lineEnd, deleteList)
        mainStart = int(np.mean(startFiltered))
        mainEnd = int(np.mean(endFiltered))

        return startFiltered, endFiltered, mainStart, mainEnd


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()