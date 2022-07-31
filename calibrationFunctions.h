#ifndef PROCESSING_H
#define PROCESSING_H

#include <stdio.h>
#include <opencv.hpp>
#include <calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void extract_corners(Mat frame, Mat &corner_detcted, Size pattern_size, vector<Point2f> &corner_set);
void record_coordinates(vector<Vec3f> &point_set, vector<vector<Vec3f>> &point_list, vector<Point2f> corner_set, vector<vector<Point2f>> &corner_list);
void append_parameters_csv(char *filename, Mat matrix, int reset_file);
int read_image_data_csv(char *filename, vector<vector<float>> &data);
void calibrate_camera(char* csv_file, double &RMS_reprojection_error, vector<vector<Vec3f>> point_list, vector<vector<Point2f>> corner_list, Mat frame, Mat &camera_matrix, Mat &dis_coef, vector<Mat> &rotate_vec, vector<Mat> &tran_vec);
void calculate_metrices(char* csv_file, vector<Vec3f> &point_set, vector<Point2f> corner_set, Mat &camera_matrix_read, Mat &dis_coef_read, Mat &PNP_rotate_vec, Mat &PNP_tran_vec);
void Harris_corners(Mat src, Mat &dst, int block_size, int aperture_size, double k, int threshold);
int read_obj_file(char *file_name, vector<Vec3f> &v_vec, vector<int> &v_idx);

#endif
