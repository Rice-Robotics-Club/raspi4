import apriltag
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True)
args = vars(ap.parse_args())


# load the input image and convert it to grayscale
print("[INFO] loading image...")
image = cv2.imread(args["image"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
results = detector.detect(gray)
camera_params = [633.40, 631.73, 318.52, 247.11]
# .173 m = tagsize, cx & cy = 960 & 540, Resolution (pixel): 1920x1080
# HFOV = 70.42 deg, VFOV = 33.54, DFOV = 78
# fx = (image_width_in_pixels * 0.5) / tan(HFOV * 0.5 * PI/180) = 
# fy = (image_height_in_pixels * 0.5) / tan(VFOV * 0.5 * PI/180)
# IGNORE ALL ABOVE: fx = 633.40, fy = 631.73, cx = 318.52, cy = 247.11
print(detector.detection_pose(results[0],camera_params, 0.173))
print("[INFO] {} total AprilTags detected".format(len(results)))


# loop over the AprilTag detection results
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