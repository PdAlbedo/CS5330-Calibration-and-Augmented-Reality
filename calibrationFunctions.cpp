/**
 * @file calibrationFunctions.cpp
 * @author Xichen Liu
 * @brief 
 * Includes the functions used in CalibrationNAR.cpp
 */


#include <stdio.h>
#include <opencv.hpp>
#include <calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

/**
 * @brief Detect and Extract Chessboard Corners
 * 
 * @param frame         video frame
 * @param pattern_size  size of corners
 * @param corner_set    poxision of corners
 */
void extract_corners(Mat frame, Mat &corner_detcted, Size pattern_size, vector<Point2f> &corner_set) {
    corner_set.clear();
    bool pattern_found = findChessboardCorners(frame, pattern_size, corner_set);
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    if (pattern_found) {
        cornerSubPix(gray, corner_set, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 50, 0.1));
    }
    drawChessboardCorners(corner_detcted, pattern_size, Mat(corner_set), pattern_found);
    imshow("Detect and Extract Chessboard Corners", corner_detcted);
}

/**
 * @brief Save the corner locations and the corresponding 3D world points.
 * 
 * @param point_set     world coordinates of a single image
 * @param point_list    world coordinates of all calibration images
 * @param corner_set    corner coordinates of a single image
 * @param corner_list   corner coordinates of all calibration images
 */
void record_coordinates(vector<Vec3f> &point_set, vector<vector<Vec3f>> &point_list, 
                        vector<Point2f> corner_set, vector<vector<Point2f>> &corner_list) {
    
    int point_list_size = point_list.size();
    int corner_list_size = corner_list.size();

    cout << endl << "Frame " << corner_list.size() << ":" << endl;

    point_set.clear();
    for (int i = 0; i > -6; i--) {
        for (int j = 0; j < 9; j++) {
            point_set.push_back(Vec3f(j, i, 0));
        }
    }
    point_list.push_back(point_set);
    corner_list.push_back(corner_set);

    cout << endl << "World coordinates:" << endl;
    for (int j = 0; j < point_list[point_list_size].size(); j++) {
        cout << point_list[point_list_size][j] << " ";
        if ((j + 1) % 9 == 0) cout << endl;
    }
    cout << "corner coordinates:" << endl;
    for (int j = 0; j < corner_list[corner_list_size].size(); j++) {
        cout << corner_list[corner_list_size][j] << " ";
        if ((j + 1) % 9 == 0) cout << endl;
    }
    cout << "Finish Recording" << endl;
}

/**
 * @brief Append the camera matrix and the distortion ceofficients to csv file
 * 
 * @param filename      name of csv file name
 * @param matrix        input parameters
 * @param reset_file    determine whether the current content of csv will be earsed
 */
void append_parameters_csv(char *filename, Mat matrix, int reset_file) {
    char mode[8];
    FILE *fp;

    strcpy(mode, "a");

    if (reset_file) {
        strcpy(mode, "w");
    }

    fp = fopen(filename, mode);
    if (!fp) {
        printf("Unable to open output file %s\n", filename);
        exit(-1);
    }

    for (int i = 0; i < matrix.rows; i++) {
        for (int j = 0; j < matrix.cols; j++) {
            if (i == 0 && j == 0) {
                char buffer[256];
                sprintf(buffer, "%.4f", matrix.at<double>(i, j));
                fwrite(buffer, sizeof(char), strlen(buffer), fp);
            }
            else {
                char buffer[256];
                sprintf(buffer, ",%.4f", matrix.at<double>(i, j));
                fwrite(buffer, sizeof(char), strlen(buffer), fp);
            }
        }
    }
        
    fwrite("\n", sizeof(char), 1, fp); // EOL

    fclose(fp);
}

int getstring(FILE *fp, char os[]) {
    int p = 0;
    int eol = 0;
    
    for (;;) {
        char ch = fgetc(fp);
        if (ch == ',') {
            break;
        }
        else if (ch == '\n' || ch == EOF) {
            eol = 1;
            break;
        }
        // printf("%c", ch ); // uncomment for debugging
        os[p] = ch;
        p++;
    }
    
    // printf("\n"); // uncomment for debugging
    os[p] = '\0';

    return eol; // return true if eol
}

int getint(FILE *fp, int *v) {
    char s[256];
    int p = 0;
    int eol = 0;

    for (;;) {
        char ch = fgetc(fp);

        if (ch == ',') {
            break;
        }
        else if (ch == '\n' || ch == EOF) {
            eol = 1;
            break;
        }
        
        s[p] = ch;
        p++;
    }

    s[p] = '\0'; // terminator
    *v = atoi(s);

    return eol; // return true if eol
}

int getfloat(FILE *fp, float *v) {
    char s[256];
    int p = 0;
    int eol = 0;

    for (;;) {
        char ch = fgetc(fp);

        if (ch == ',') {
            break;
        }
        else if (ch == '\n' || ch == EOF) {
            eol = 1;
            break;
        }
        
        s[p] = ch;
        p++;
    }

    s[p] = '\0'; // terminator
    *v = atof(s);

    return eol; // return true if eol
}

/**
 * @brief Read camera matrix and distortion coefficients from the csv file
 * 
 * @param filename  name of csv file
 * @param data      vector that stores camera matrix and distortion coefficients
 */
int read_image_data_csv(char *filename, vector<vector<float>> &data) {
    FILE *fp;
    float fval;
    int iter = 0;

    fp = fopen(filename, "r");
    if (!fp) {
        printf("Unable to open feature file\n");
        return(-1);
    }

    for (;;) {
        vector<float> dvec;

        for (;;) {
            // get next feature
            float eol = getfloat(fp, &fval);
            dvec.push_back(fval);
            if (eol) {
                iter++;
                break;
            }
        }

        data.push_back(dvec);

        // stop after the 2nd iteration, because only camera matrix and distortion coefficients need to be extracted 
        if (iter == 2) break;
    }
    fclose(fp);

    return 0;
}

/**
 * @brief Calibrate the Camera
 * 
 * @param RMS_reprojection_error    The overall RMS re-projection error.
 * @param point_list                Points in the world coordinates
 * @param corner_list               Corners in the image coordinates
 * @param frame                     Image
 * @param camera_matrix             Input/output 3x3 floating-point camera intrinsic matrix
 * @param dis_coef                  Input/output vector of distortion coefficients
 * @param rotate_vec                Output vector of rotation vectors (Rodrigues) estimated for each pattern view
 * @param tran_vec                  Output vector of translation vectors estimated for each pattern view
 */
void calibrate_camera(char* csv_file, double &RMS_reprojection_error, vector<vector<Vec3f>> point_list, 
                        vector<vector<Point2f>> corner_list, Mat frame, Mat &camera_matrix, 
                        Mat &dis_coef, vector<Mat> &rotate_vec, vector<Mat> &tran_vec) {

    RMS_reprojection_error = calibrateCamera(point_list, corner_list, frame.size(), camera_matrix, 
                                                dis_coef, rotate_vec, tran_vec, CALIB_FIX_ASPECT_RATIO);

    cout << endl << "RMS re-projection error: " <<  RMS_reprojection_error << endl;
    cout << "Camera matrix:" << endl;
    for (int i = 0; i < camera_matrix.rows; i++) {
        for (int j = 0; j < camera_matrix.cols; j++) {
            cout << camera_matrix.at<double>(i, j) << "\t";
        }
        cout << endl;
    }
    cout << "Distortion coefficient: " << dis_coef << endl;

    append_parameters_csv(csv_file, camera_matrix, 1);
    append_parameters_csv(csv_file, dis_coef, 0);
}

/**
 * @brief Calculate Current Position of the Camera
 * 
 * @param csv_file              csv file that stores the info of camera matrix and distortion coefficient
 * @param point_set             real world coordinates
 * @param corner_set            corner coordinates in image
 * @param camera_matrix_read    camera matrix read from csv file
 * @param dis_coef_read         distortion coefficient read from csv file
 * @param PNP_rotate_vec        rotation matrix
 * @param PNP_tran_vec          translation matrix
 */
void calculate_metrices(char* csv_file, vector<Vec3f> &point_set, vector<Point2f> corner_set, 
                        Mat &camera_matrix_read, Mat &dis_coef_read, Mat &PNP_rotate_vec, Mat &PNP_tran_vec) {
    // read camera matrix and distortion coefficient from csv file
    double camera_matrix_2Darray_read[3][3];
    double dis_coef_2Darray_read[1][5];
    vector<vector<float>> parameters;

    read_image_data_csv(csv_file, parameters);
    int c = 0;
    while (c < parameters[0].size()) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                camera_matrix_2Darray_read[i][j] = parameters[0][c];
                c++;
            }
        }
    }
    c = 0;
    while (c < parameters[1].size()) {
        for (int i = 0; i < 1; i++) {
            for (int j = 0; j < 5; j++) {
                dis_coef_2Darray_read[i][j] = parameters[1][c];
                c++;
            }
        }
    }
    camera_matrix_read = Mat(3, 3, CV_64FC1, &camera_matrix_2Darray_read);
    dis_coef_read = Mat(1, 5, CV_64FC1, &dis_coef_2Darray_read);

    point_set.clear();
    for (int i = 0; i > -6; i--) {
        for (int j = 0; j < 9; j++) {
            point_set.push_back(Vec3f(j, i, 0));
        }
    }
    
    solvePnP(point_set, corner_set, camera_matrix_read, dis_coef_read, PNP_rotate_vec, PNP_tran_vec);

    camera_matrix_read = camera_matrix_read.clone();
    dis_coef_read = dis_coef_read.clone();
}

/**
 * @brief Circle the corners detected by Harris corner detector 
 * 
 * @param src               input single-channel 8-bit image
 * @param dst               image to store the Harris detector responses
 * @param block_size        neighborhood size
 * @param aperture_size     aperture parameter for the Sobel operator.
 * @param k                 Harris detector free parameter
 * @param threshold         the threshold determines whether a feature point will be kept
 */
void Harris_corners(Mat src, Mat &dst, int block_size, int aperture_size, double k, int threshold) {
    cornerHarris(src, dst, block_size, aperture_size, k);
    Mat dst_norm, dst_norm_scaled;
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);

    for (int i = 0; i < dst_norm.rows; i++) {
        for (int j = 0; j < dst_norm.cols; j++) {
            if ((int) dst_norm.at<float>(i, j) > threshold) {
                circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2);
            }
        }
    }
    dst = dst_norm_scaled.clone();
}

/**
 * @brief Read positions of the vertices
 * 
 * @param file_name     obj file
 * @param v_vec         position of vertices
 * @param v_idx         surfaces that consist of vertices
 */
int read_obj_file(char *file_name, vector<Vec3f> &v_vec, vector<int> &v_idx) {
    FILE *file = fopen(file_name, "r");
    if( file == NULL ){
        printf("Impossible to open the file !n");
        return (-1);
    }
    while(true){
        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break;
        if (strcmp( lineHeader, "v") == 0){
            Vec3f vertex;
            fscanf(file, "%f %f %f/n", &vertex[0], &vertex[2], &vertex[1]);
            v_vec.push_back(vertex);
        }
        else if (strcmp(lineHeader, "f") == 0 ){
            std::string vertex1, vertex2, vertex3;
            unsigned int vertexIndex[3];
            int matches = fscanf(file, "%d %d %d/n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2]);
            if (matches != 3){
                printf("File can't be read by our simple parser : (Try exporting with other optionsn");
                return false;
            }
            v_idx.push_back(vertexIndex[0]);
            v_idx.push_back(vertexIndex[1]);
            v_idx.push_back(vertexIndex[2]);
        }
    }
    return 0;
}
