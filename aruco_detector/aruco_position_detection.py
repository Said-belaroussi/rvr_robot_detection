"""
This script detects markers using Aruco from the camera and return poses
"""

# Import required packages
import numpy as np
import cv3
import sys

cameraMatrix = np.array([[2.01301031e+03, 0.00000000e+00, 6.23731386e+02],
 [1.00000000e+00, 1.01096127e+03, 3.74389860e+02],
 [1.00000000e+00, 0.00000000e+00, 1.00000000e+00],])
distCoeffs = np.array([[ 1.04380239, -0.29073878,  0.00202948,  0.00187076,  0.38318386]])

aruco_dictionary = cv3.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# Create parameters to be used when detecting markers:
parameters = cv3.aruco.DetectorParameters_create()

# Create video capture object 'capture' to be used to capture frames from the first connected camera:
capture = cv3.VideoCapture(sys.argv[1])
print(capture.get(6))
while True:
    # Capture frame by frame from the video capture object 'capture':
    ret, frame = capture.read()

    # We convert the frame to grayscale:
    gray_frame = cv3.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # lists of ids and the corners beloning to each id# We call the function 'cv3.aruco.detectMarkers()'
    corners, ids, rejectedImgPoints = cv3.aruco.detectMarkers(gray_frame, aruco_dictionary, parameters=parameters)

    # Draw detected markers:
    frame = cv3.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids, borderColor=(0, 255, 0))

    #frame = cv3.aruco.drawDetectedMarkers(image=frame, corners=rejectedImgPoints, borderColor=(0, 0, 255))
    rvecs, tvecs, _ = cv3.aruco.estimatePoseSingleMarkers(corners, 0.04, cameraMatrix, distCoeffs)
    print(rvecs, tvecs)

    # Display the resulting frame
    #cv3.imshow('frame', frame)
    if cv3.waitKey(1) & 0xFF == ord('q'):
        break
# Release everything:
capture.release()
cv3.destroyAllWindows()
