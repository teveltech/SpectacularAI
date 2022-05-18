#include "ros/ros.h"
#include "sharedmemtool.hpp"
#include <iostream>
//
// Created by root on 5/18/22.
//

int main() {

    CameraReader* reader = new CameraReader();
    reader->readFrame();



    delete reader;
    std::cout << "test" << std::endl;
    return 0;
}
