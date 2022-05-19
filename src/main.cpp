#include "ros/ros.h"
#include "sharedmemtool.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
//
// Created by root on 5/18/22.
//

using namespace cv;

int main() {

    CameraReader* reader = new CameraReader();

    namedWindow("rgb"); // Create a window
    namedWindow("depth"); // Create a window

    while (true) {
        reader->readFrame();
        imshow("rgb", reader->frame); // Show our image inside the created window.
        imshow("depth", reader->D435_cam->m_cvMatDepthFrame); // Show our image inside the created window.

        waitKey(10);
    }
    destroyWindow("rgb"); //destroy the created window
    destroyWindow("depth"); //destroy the created window

    delete reader;
    std::cout << "test" << std::endl;
    return 0;
}
