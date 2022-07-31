/**
 * @file harrisCornerDetector.cpp
 * @author Xichen Liu
 * @brief 
 * Detect Robust Features by using Harris corner detector
 */

#include <stdio.h>
#include <opencv.hpp>
#include <calib3d/calib3d.hpp>
#include "calibrationFunctions.cpp"

using namespace std;
using namespace cv;

int main (int argc, char* argv[]) {

    VideoCapture *capdev;

    // open the video device
    capdev = new VideoCapture(0);
    if(!capdev -> isOpened()) {
            printf("Unable to open video device\n");
            return(-1);
    }
    // identifies a window
    // namedWindow("Video", WINDOW_NORMAL);
    Mat frame;
    // Mat frame = imread("checkerboard.png");
    Mat gray_img;
    int block_size = 2;
    int aperture_size = 3;
    double k = 0.04;
    int threshold = 150;
    bool isHarris = false;
    
    while (true) {
        // get a new frame from the camera, treat as a stream
        *capdev >> frame;
        if(frame.empty()) {
            printf("frame is empty\n");
            break;
        }
        imshow("Video", frame);

        if (isHarris) {
            cvtColor(frame, gray_img, COLOR_BGR2GRAY);
            Mat dst = Mat::zeros(gray_img.size(), CV_32FC1);
            Harris_corners(gray_img, dst, block_size, aperture_size, k, threshold);
            imshow("Harris corners detected", dst);
        }

        char k = waitKey(3);
        if (k == 'q') break;
        else if (k == 'h') {
            isHarris = true;
        }
    }


    return 0;
}