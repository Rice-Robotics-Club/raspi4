import cv2
import apriltag
# import time

# Initialize the camera (0 is usually the default camera)
cam = cv2.VideoCapture(0)

# Check if the camera is opened correctly
if not cam.isOpened():
    raise IOError("Cannot open webcam")

# time.sleep(.25)
for i in range(5):
    ret, frame = cam.read()


# Capture a frame
# ret, frame = cam.read()

# If frame is captured without errors
if ret:
    #frame contains image
    image = frame
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



    options = apriltag.DetectorOptions(families="tag36h11")
    #options = apriltag.DetectorOptions(refine_edges=False, quad_contours=False) #to address to many contours
    detector = apriltag.Detector(options)
    results = detector.detect(gray)


    if (len(results) == 0):
        print("No AprilTag")
        exit(1)

    camera_params = [633.40, 631.73, 318.52, 247.11]
    matrix, error1, error2 = detector.detection_pose(results[0],camera_params, 0.173)
    print(matrix)
    Tx, Ty, Tz = matrix[0, 3], matrix[1, 3], matrix[2, 3] 
    print(f"Position right {Tx:.3f}m, up {Ty:.3f}m, forward {Tz:.3f}m")     
    print("[INFO] {} total AprilTags detected".format(len(results)))

    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("[INFO] tag family: {}".format(tagFamily))

    # show the output image after AprilTag detection
    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Failed to capture image")


# Release the camera
cam.release()