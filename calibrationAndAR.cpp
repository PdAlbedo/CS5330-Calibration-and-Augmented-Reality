/**
 * @file calibrationAndAR.cpp
 * @author Xichen Liu
 * @brief 
 * Program that can detect a target and then place a virtual object in the scene relative to the target 
 * that moves and orients itself correctly given motion of the camera or target.
 */


#include <stdio.h>
#include <opencv.hpp>
#include <calib3d/calib3d.hpp>
// #include "calibrationFunctions.cpp"
#include "calibrationFunctions.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    VideoCapture *capdev;
    string csv_name = "data.csv";
    char* csv_file = &csv_name[0];

    Size pattern_size(9, 6);
    vector<Point2f> corner_set;
	vector<vector<Point2f>> corner_list;
    vector<Vec3f> point_set;
	vector<vector<Vec3f>> point_list;
    
    vector<Vec3f> v_vec;
    vector<int> v_idx;

    bool isCalibrated = false;
    bool positionCalculated = false;
    bool is3DAxes = false;
    bool isProjected = false;
    bool isCow = false;
    bool isStatic = false;

    Mat camera_matrix_read;
    Mat dis_coef_read;
    Mat PNP_rotate_vec;
    Mat PNP_tran_vec;

    // open the video device
    capdev = new VideoCapture(0);
    if(!capdev -> isOpened()) {
            printf("Unable to open video device\n");
            return(-1);
    }
    // identifies a window
    // namedWindow("Video", WINDOW_NORMAL);
    Mat frame;
    Mat corner_detcted;
    Mat axes_img;
    Mat projected_img;
    Mat projected_cow;

    cout << "Press 's' to store the current frame as a calibration image" << endl
            << "After at least 5 calibration images stored, press 'c' to calibrate the camera" << endl
            << "After calibrating the camera, press 'p' to calculate the rotation matrix and translation matrix" << endl
            << "After calculateing the rotation matrix and translation matrix, press 'a' to show 3D axes" << endl
            << "After calculateing the rotation matrix and translation matrix, press 'v' to show the virtual object" << endl
            << "After calculateing the rotation matrix and translation matrix, press 'o' to place the virtual cow" << endl // TODO:
            << "Press 'q' to quit" << endl << endl;

    while (true) {
        // get a new frame from the camera, treat as a stream
        *capdev >> frame;
        if(frame.empty()) {
            printf("frame is empty\n");
            break;
        }
        imshow("Video", frame);

        // Task 1: Detect and Extract Chessboard Corners
        corner_detcted = frame.clone();
        extract_corners(frame, corner_detcted, pattern_size, corner_set);

        if (is3DAxes) {
            axes_img = frame.clone();
            PNP_rotate_vec.release();
            PNP_tran_vec.release();
            camera_matrix_read.release();
            dis_coef_read.release();
            if (corner_set.size() != 54) continue;

            calculate_metrices(csv_file, point_set, corner_set, camera_matrix_read, dis_coef_read, PNP_rotate_vec, PNP_tran_vec);
            vector<Vec3f> real_world;
            real_world.push_back(Vec3f(0, 0, 0));
            real_world.push_back(Vec3f(0, -3, 0));
            real_world.push_back(Vec3f(3, 0, 0));
            real_world.push_back(Vec3f(0, 0, 3));

            vector<Point2f> image_points;
            projectPoints(real_world, PNP_rotate_vec, PNP_tran_vec, camera_matrix_read, dis_coef_read, image_points);
            line(axes_img, image_points[0], image_points[1], Scalar(0, 0, 255), 2);
            line(axes_img, image_points[0], image_points[2], Scalar(0, 255, 0), 2);
            line(axes_img, image_points[0], image_points[3], Scalar(255, 0, 0), 2);
            imshow("3D Axes", axes_img);

        }

        if (isProjected) {
            projected_img = frame.clone();
            PNP_rotate_vec.release();
            PNP_tran_vec.release();
            camera_matrix_read.release();
            dis_coef_read.release();
            if (corner_set.size() != 54) continue;

            calculate_metrices(csv_file, point_set, corner_set, camera_matrix_read, dis_coef_read, PNP_rotate_vec, PNP_tran_vec);
            vector<Vec3f> real_world;
            real_world.push_back(Vec3f(1, -1, 0));
            real_world.push_back(Vec3f(1, -5, 0));
            real_world.push_back(Vec3f(5, -1, 0));
            real_world.push_back(Vec3f(5, -5, 0));
            real_world.push_back(Vec3f(3, -3, 4));

            vector<Point2f> image_points;
            projectPoints(real_world, PNP_rotate_vec, PNP_tran_vec, camera_matrix_read, dis_coef_read, image_points);
            for (int i = 0; i < image_points.size(); i++) {
                for (int j = 0; j < image_points.size(); j++) {
                    line(projected_img, image_points[i], image_points[j], Scalar(0, 0, 255), 2);
                }
            }
            imshow("Virtual Object", projected_img);
        }

        if (isCow){
            projected_cow = frame.clone();
            PNP_rotate_vec.release();
            PNP_tran_vec.release();
            camera_matrix_read.release();
            dis_coef_read.release();
            if (corner_set.size() != 54) continue;

            calculate_metrices(csv_file, point_set, corner_set, camera_matrix_read, dis_coef_read, PNP_rotate_vec, PNP_tran_vec);

            vector<Point2f> image_points;
            projectPoints(v_vec, PNP_rotate_vec, PNP_tran_vec, camera_matrix_read, dis_coef_read, image_points);
            for (int i = 0; i < v_idx.size(); i+=3) {
                line(projected_cow, image_points[v_idx[i] - 1], image_points[v_idx[i + 1] - 1], Scalar(0, 0, 255), 1);
                line(projected_cow, image_points[v_idx[i + 1] - 1], image_points[v_idx[i + 2] - 1], Scalar(0, 0, 255), 1);
                line(projected_cow, image_points[v_idx[i + 2] - 1], image_points[v_idx[i] - 1], Scalar(0, 0, 255), 1);
            }
            imshow("Cow", projected_cow);
        }

        char k = waitKey(3);
        if (k == 'q') {
            break;
        }
        // Task 2: Save the corner locations and the corresponding 3D world points.
        else if (k == 's') {
            record_coordinates(point_set, point_list, corner_set, corner_list);
        }
        // Task 3: Calibrate the Camera
        else if (k == 'c') {
            if (corner_list.size() < 5) {
                cout << "No enough calibration images input, at least 5 images needed. Current # of images: " << corner_list.size() << endl;
            }
            else {
                double camera_matrix_2Darray[3][3] = {
                                                        {1, 0, (double)frame.cols/2},
                                                        {0, 1, (double)frame.rows/2}, 
                                                        {0, 0, 1}};
                Mat camera_matrix = Mat(3, 3, CV_64FC1, &camera_matrix_2Darray);
                double RMS_reprojection_error;
                Mat dis_coef;
                vector<Mat> rotate_vec;
                vector<Mat> tran_vec;

                calibrate_camera(csv_file, RMS_reprojection_error, point_list, corner_list, frame, camera_matrix, dis_coef, rotate_vec, tran_vec);
                isCalibrated = true;
            }
        }
        // Task 4: Calculate Current Position of the Camera
        else if (k == 'p') {
            if (isCalibrated) {
                calculate_metrices(csv_file, point_set, corner_set, camera_matrix_read, dis_coef_read, PNP_rotate_vec, PNP_tran_vec);
                positionCalculated = true;

                cout << endl << "Camera matrix read:" << endl;
                for (int i = 0; i < camera_matrix_read.rows; i++) {
                    for (int j = 0; j < camera_matrix_read.cols; j++) {
                        cout << camera_matrix_read.at<double>(i, j) << "\t";
                    }
                    cout << endl;
                }
                cout << "Distortion coefficient read: " << dis_coef_read << endl;
                cout << endl << "Rotation matrix: " << endl;
                for (int i = 0; i < PNP_rotate_vec.rows; i++) {
                    for (int j = 0; j < PNP_rotate_vec.cols; j++) {
                        cout << "    " << PNP_rotate_vec.at<double>(i, j) << endl;
                    }
                }
                cout << "Translation matrix: " << endl;
                for (int i = 0; i < PNP_tran_vec.rows; i++) {
                    for (int j = 0; j < PNP_tran_vec.cols; j++) {
                        cout << "    " << PNP_tran_vec.at<double>(i, j) << endl;
                    }
                }

            }
            else {
                cout << "Camera is not calibrated, press 'c' to calibrate the camera" << endl;
            }
        }
        // Task 5: Project Outside Corners or 3D Axes
        else if (k == 'a') {
            if(positionCalculated) {
                is3DAxes = true;
            }
            else {
                cout << "Rotation matrix and translation matrix is not calculated, press 'p' to calculate" << endl; 
            }
        }
        // Task 6: Create a Virtual Object
        else if (k == 'v') {
            if(positionCalculated) {
                isProjected = true;
            }
            else {
                cout << "Rotation matrix and translation matrix is not calculated, press 'p' to calculate" << endl; 
            }
        }
        // Extension: deal with static images
        else if (k == 'o') {
            if(positionCalculated) {
                string obj_name = "cow.obj";
                char *obj_file = &obj_name[0];
                read_obj_file(obj_file, v_vec, v_idx);
                double min_0 = 99;
                double max_1 = -99;
                double min_2 = 99;
                // add some offset to make it at the middle
                for (Vec3f &it: v_vec) {
                    if (it[0] < min_0) min_0 = it[0];
                    if (it[1] > max_1) max_1 = it[1];
                    if (it[2] < min_2) min_2 = it[2];
                }
                for (auto &it: v_vec) {
                    it[0] += abs(min_0);
                    it[1] -= max_1;
                    it[2] += abs(min_2);
                }
                isCow = true;
            }
            else {
                cout << "Rotation matrix and translation matrix is not calculated, press 'p' to calculate" << endl;
            }
        }
    }

    return 0;
}
