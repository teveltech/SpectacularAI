//
// Created by root on 3/21/21.
//

#include "cam_params.h"
#include "farConfigParser.hpp"
#include <opencv2/core/mat.hpp>
#include "CameraParameters.hpp"

using namespace std;
cam_params::cam_params() {}

cam_params::cam_params(farConfigParser config_obj, CameraIntrinsics intrinsics, int cam_type)
{
    switch (cam_type) {
        case 0: //uvc
        readFromJsonFile(config_obj);
        break;
        case 1: //D435
        readParamsFromCamera(intrinsics);
        break;
        default:
            utilities::loggerUtility::writeLog(TVL_LOG_FATAL, "cam_params::cam_params(), camera type is undefined");

    }



}

void cam_params::init(CameraIntrinsics intrinsics)
{
    readParamsFromCamera(intrinsics);
}

void cam_params::init(farConfigParser config_obj)
{
    readFromJsonFile(config_obj);
}

cam_params::~cam_params()
{

}

void cam_params::readFromJsonFile(farConfigParser config_obj)
{
    cv::Mat MCamera ;

    auto cameraDevices = config_obj.getCameraFrontEnd()->getAllDevices();
    std::map<unsigned int, std::map<unsigned int, float> > posMat;
    bool isIntrinsics;
    Json::Value _intrinsics;
    string cameraType;
    float fx, fy, cx, cy, gamma=0.0;
    for (auto it = cameraDevices.begin(); it != cameraDevices.end(); ++it)
    {
        if(it->find("UVC") != std::string::npos)
        {
            //#TODO print uvc matrix and distortion to log
            cameraType = config_obj.getCameraFrontEnd()->getDeviceConfig(*it)->getJsonData()["HEADER"]["SERIAL_NUM"].asString();
            posMat = std::dynamic_pointer_cast<cameraConfig>(config_obj.getCameraFrontEnd()->getDeviceConfig(*it))->getPositionalMatrix();
            if (config_obj.getCameraFrontEnd()->getDeviceConfig(*it)->getJsonData()["HEADER"].isMember("INTRINSICS"))
            {
                _intrinsics = config_obj.getCameraFrontEnd()->getDeviceConfig(*it)->getJsonData()["HEADER"]["INTRINSICS"];
                fx = _intrinsics[0].asFloat();
                fy = _intrinsics[1].asFloat();
                cx = _intrinsics[2].asFloat();
                cy = _intrinsics[3].asFloat();
                if(cameraType == "ATF")
                {
                    gamma = _intrinsics[4].asFloat();
                }
            } else
            {
                fx = posMat[0][0];
                fy = posMat[1][1];
                cx = posMat[0][2];
                cy = posMat[1][2];
                gamma = posMat[0][1];
            }

            break;
        }
    }
    if (posMat.size() != 4 || posMat[0].size() != 4 || posMat[1].size() != 4 || posMat[2].size() != 4 || posMat[3].size() != 4)
    {
        //TODO
        utilities::loggerUtility::writeLog(TVL_LOG_FATAL, "cam_params::readFromJsonFile(), Uvc camera matrix not valid!"
                                                          "check systemConfig.json.");
        throw;
        //error handling!!!
    }
    float data[9] = {fx,  gamma,  cx,
                     0.0, fy,   cy,
                     0.0, 0.0,  1.0};

    MCamera = cv::Mat(3,3, CV_32F, data);
    if (MCamera.type() != CV_32FC1)
        MCamera.convertTo(CameraMatrix, CV_32FC1);
    else
        MCamera.copyTo(CameraMatrix);

    utilities::loggerUtility::writeLog(TVL_LOG_INFO, "cam_params::readFromJsonFile(), Uvc camera intrinsics: "
                                                     "fx=%f, fy=%f, cx=%f, cy=%f, gamma=%f", CameraMatrix.at<float>(0, 0),
                                                     CameraMatrix.at<float>(1, 1), CameraMatrix.at<float>(0, 2),
                                                     CameraMatrix.at<float>(1, 2), CameraMatrix.at<float>(0, 1));

    if(cameraType == "ATF")
    {
        getCameraDistortion(config_obj);
    } else{
        getCameraDistortion(config_obj);
    }
}

void cam_params::getCameraDistortion(farConfigParser config_obj)
{
    cv::Mat MDist;
    auto uvcDist0 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][0].asFloat();
    auto uvcDist1 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][1].asFloat();
    auto uvcDist2 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][2].asFloat();
    auto uvcDist3 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][3].asFloat();
    auto uvcDist4 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][4].asFloat();
    float mdata[5] = {uvcDist0, uvcDist1, uvcDist2, uvcDist3, uvcDist4};
    MDist = cv::Mat(1, 5, CV_32F, mdata);
    if (MDist.total() < 5){
        utilities::loggerUtility::writeLog(TVL_LOG_FATAL, "cam_params::readFromJsonFile(), Uvc distortion polynom not valid!"
                                                          "check systemConfig.json.");
        throw;
    }
    // convert to 32 and get the 4 first elements only
    cv::Mat mdist32;
    MDist.convertTo(mdist32, CV_32FC1);

    Distorsion.create(1, 5, CV_32FC1);
    for (int i = 0; i != 5; i++)
        Distorsion.ptr<float>(0)[i] = mdist32.ptr<float>(0)[i];

    utilities::loggerUtility::writeLog(TVL_LOG_INFO, "cam_params::readFromJsonFile(), Uvc camera Distortion: "
                                                     "%f, %f, %f, %f, %f", Distorsion.at<float>(0),
                                                     Distorsion.at<float>(1), Distorsion.at<float>(2),
                                                     Distorsion.at<float>(3), Distorsion.at<float>(4));
}

void cam_params::getCameraDistortion(farConfigParser config_obj, bool flag)
{
    cv::Mat MDist;
    auto uvcDist0 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][0].asFloat();
    auto uvcDist1 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][1].asFloat();
    auto uvcDist2 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][2].asFloat();
    auto uvcDist3 = config_obj.getJsonData()["APPLICATIONS"]["ODOMETRY_PARAMS"]["UVC_DIST"][3].asFloat();
    float mdata[4] = {uvcDist0, uvcDist1, uvcDist2, uvcDist3};
    MDist = cv::Mat(1, 4, CV_32F, mdata);
    if (MDist.total() != 4){
        utilities::loggerUtility::writeLog(TVL_LOG_FATAL, "cam_params::readFromJsonFile(), Uvc distortion polynom not valid!"
                                                          "check systemConfig.json.");
        throw;
    }
    // convert to 32 and get the 4 first elements only
    cv::Mat mdist32;
    MDist.convertTo(mdist32, CV_32FC1);

    Distorsion.create(1, 4, CV_32FC1);
    for (int i = 0; i < 4; i++)
        Distorsion.ptr<float>(0)[i] = mdist32.ptr<float>(0)[i];

    utilities::loggerUtility::writeLog(TVL_LOG_INFO, "cam_params::readFromJsonFile(), Uvc camera Distortion: "
                                                     "%f, %f, %f, %f", Distorsion.at<float>(0),
                                                     Distorsion.at<float>(1), Distorsion.at<float>(2),
                                                     Distorsion.at<float>(3));
}

void cam_params::readParamsFromCamera(CameraIntrinsics intrinsics)
{
    cv::Mat MCamera, MDist;

    float data[9] = {intrinsics.fx, 0.0,           intrinsics.ppx,
                     0.0,           intrinsics.fy, intrinsics.ppy,
                     0.0, 0.0, 1.0};
    MCamera = cv::Mat(3,3, CV_32F, data);

    if (MCamera.type() != CV_32FC1)
        MCamera.convertTo(CameraMatrix, CV_32FC1);
    else
        MCamera.copyTo(CameraMatrix);

    float mdata[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    MDist = cv::Mat(1, 5, CV_32F, mdata);
    if (MDist.total() < 5){
        utilities::loggerUtility::writeLog(TVL_LOG_FATAL, "cam_params::readParamsFromCamera(), Uvc distortion polynom not valid!"
                                                          "check systemConfig.json.");
        throw;
    }
    // convert to 32 and get the 4 first elements only
    cv::Mat mdist32;
    MDist.convertTo(mdist32, CV_32FC1);

    Distorsion.create(1, 5, CV_32FC1);
    for (int i = 0; i < 5; i++)
        Distorsion.ptr<float>(0)[i] = mdist32.ptr<float>(0)[i];

    utilities::loggerUtility::writeLog(TVL_LOG_INFO, "cam_params::readParamsFromCamera(), D435 camera intrinsics: "
                                                     "fx=%f, fy=%f, cx=%f, cy=%f", CameraMatrix.at<float>(0, 0),
                                                     CameraMatrix.at<float>(1, 1), CameraMatrix.at<float>(0, 2),
                                                     CameraMatrix.at<float>(1, 2));

    utilities::loggerUtility::writeLog(TVL_LOG_INFO, "cam_params::readParamsFromCamera(), D435 camera Distortion: "
                                                     "%f, %f, %f, %f, %f", Distorsion.at<float>(0),
                                                     Distorsion.at<float>(1), Distorsion.at<float>(2),
                                                     Distorsion.at<float>(3), Distorsion.at<float>(4));

}