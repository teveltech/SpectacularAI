//
// Created by root on 3/21/21.
//

#ifndef ARUCO_READER_CAM_PARAMS_H
#define ARUCO_READER_CAM_PARAMS_H

#include "farConfigParser.hpp"
#include <opencv2/core/mat.hpp>
#include "CameraParameters.hpp"

class cam_params {
public:
    cam_params(farConfigParser config_obj, CameraIntrinsics intrinsics, int cam_type);
    cam_params();
    ~cam_params();
    void init(farConfigParser config_obj);
    void init(CameraIntrinsics intrinsics);
    // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Mat CameraMatrix;
    //  distortion vector
    cv::Mat Distorsion;

private:
    void readFromJsonFile(farConfigParser config_obj);
    void readParamsFromCamera(CameraIntrinsics intrinsics);
    void getCameraDistortion(farConfigParser config_obj);
    void getCameraDistortion(farConfigParser config_obj, bool flag);


};


#endif //ARUCO_READER_CAM_PARAMS_H