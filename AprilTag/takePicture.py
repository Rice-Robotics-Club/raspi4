import cv2

# Initialize the camera (0 is usually the default camera)
cam = cv2.VideoCapture(0)

# Check if the camera is opened correctly
if not cam.isOpened():
    raise IOError("Cannot open webcam")

# Capture a frame
ret, frame = cam.read()

# If frame is captured without errors
if ret:
    # Display the captured frame (optional)
    cv2.imshow('Captured Image', frame)
    
    # Save the captured frame to a file
    cv2.imwrite('captured_image.jpg', frame)
    print("Image saved as captured_image.jpg")

    # Wait for a key press and then close the window
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Failed to capture image")

# Release the camera
cam.release()