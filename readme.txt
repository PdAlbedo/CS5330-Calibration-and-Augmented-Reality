CS5330 Project 4
Xichen Liu


Khoury wiki:
https://wiki.khoury.northeastern.edu/display/~xicliu/CS5330-Project-4-Report

calibrationAndAR.cpp: Main program that calibrate the camera and project objects to a chessboard
HarrisCornerDetector.cpp: Use Harris corner detector to locate the corners
calibrationFunctions.cpp/ calibrationFunctions.h: Functions used in main programs
data.csv: Stores the camera matrix and distortion coefficients from main program


Operating system: Windows 11
IDE: vscode
code-runner execute command: cd $dir && g++ $fileName calibrationFunctions.cpp -o $fileNameWithoutExt -std=c++14 -I D:\\CodeAndTools\\OpenCV\\opencv\\build\\include -I D:\\CodeAndTools\\OpenCV\\opencv\\build\\include\\opencv2 -L D:\\CodeAndTools\\OpenCV\\opencv\\build\\x64\\MinGW\\lib -l opencv_calib3d455 -l opencv_core455 -l opencv_dnn455 -l opencv_features2d455 -l opencv_gapi455 -l opencv_imgproc455 -l opencv_imgcodecs455 -l opencv_video455 -l opencv_ml455 -l opencv_highgui455 -l opencv_objdetect455 -l opencv_flann455 -l opencv_photo455 -l opencv_stitching455 -l opencv_ts455 -l opencv_videoio455 && $dir$fileNameWithoutExt


Procedure of running calibrationAndAR.cpp:

Make camera towards to the chessboard
Press 's' to save the current frame as a calibration image
After at least 5 calibration images stored, press 'c' to calibrate the camera, and show RMS reprojection error
After calibrating the camera, press 'p' to calculate the rotation matrix and translation matrix
After calculateing the rotation matrix and translation matrix, press 'a' to show 3D axes on the chessboard
After calculateing the rotation matrix and translation matrix, press 'v' to show the virtual object on the chessboard
After calculateing the rotation matrix and translation matrix, press 'o' to place the virtual cow on the chessboard
Press 'q' to quit

Procedure of running HarrisCornerDetector.cpp:

Make camera towards to the chessboard
Press 'h' to apply the Harris corner detector to the video
Press 'q' to quit