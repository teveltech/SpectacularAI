//
// Created by root on 10/24/21.
//

#ifndef __ARUCO_READER_SHARED_MEMORY_TOOL_HPP__
#define __ARUCO_READER_SHARED_MEMORY_TOOL_HPP__

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED

#include <string>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include "utilities.hpp"

#include "CameraParameters.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;
//#include "cam_params.h"
#include "sharedMemoryContainer.hpp"


class sharedMemoryFrame
        {
        public:
            sharedMemoryFrame(unsigned int _height, unsigned int _width, unsigned int _channels = 1)
            : m_height(_height)
            , m_width(_width)
            , m_channels(_channels)
            , m_frameSize(m_width * m_height * m_channels)
            , m_depthFrameSize(m_width * m_height * sizeof(float))
            {
                utilities::loggerUtility::writeLog(TVL_LOG_INFO, "");
                m_frame = new uint8_t[m_frameSize];
                m_depthFrame = new uint8_t[m_depthFrameSize];
            }
            virtual ~sharedMemoryFrame()
            {
                delete[] m_frame;
                delete[] m_depthFrame;
            }
            uint8_t* m_frame;
            uint8_t* m_depthFrame;
            unsigned int m_height;
            unsigned int m_width;
            unsigned int m_channels;
            unsigned int m_fps;
            long m_timeStamp;
            unsigned int m_frameSize;
            unsigned int m_depthFrameSize;
        };


class sharedMemoryTool
        {
        public:
            virtual ~sharedMemoryTool();
            bool loadNewFrame();
            unsigned int getHeight() { return m_frame->m_height; }
            unsigned int getWidth() { return m_frame->m_width; }
            unsigned int getChannels() { return m_frame->m_channels; }
            unsigned int getFPS() { return m_frame->m_fps; }
            long getTimestamp() { return m_frame->m_timeStamp; }
            uint8_t* getFrame() { return m_frame->m_frame; }
            uint8_t* getDepthFrame() { return m_frame->m_depthFrame; }
            void getFrameAsCvMat(cv::Mat &frame, bool rgb = true);
            long lastPrintNoFrameTime = 0;
            long lastTimeStampRead = 0;
            cv::Mat m_cvMatFrame;
            cv::Mat m_cvMatDepthFrame;
        protected:
            sharedMemoryTool();
            virtual void loadAddedDataFromSharedMemory() {}
            virtual void createFrameSharedPointer() = 0;
            bool initSM();

        protected:
            std::shared_ptr<sharedMemoryFrame> m_frame;
            std::string m_smFileName;
            std::string m_smName;
            std::string m_deviceName;
            char* m_smBuffer;
            unsigned int m_smBufferSize;
            unsigned int m_width;
            unsigned int m_height;
            unsigned int m_channels;
            std::shared_ptr<sharedMemoryContainer> m_memoryCont;
            sharedMemoryConfig m_memoryData;

            int m_cvType;
            int m_cvDepthType;
            std::string m_frameNameInSharedMemory;
            std::string m_depthFrameNameInSharedMemory;
        };

class RGBsharedMemoryFrame : public sharedMemoryFrame
        {
        public:
            RGBsharedMemoryFrame(unsigned int _height, unsigned int _width, unsigned int _channels = 1)
            : sharedMemoryFrame(_height, _width, _channels) {}
            CameraExtrinsics m_extrinsics;
            CameraIntrinsics m_intrinsics;
        };

class RGBSharedMemoryTool : public sharedMemoryTool
        {
        public: RGBSharedMemoryTool();
            void loadAddedDataFromSharedMemory() override;
            void createFrameSharedPointer() override { m_frame = std::make_shared<RGBsharedMemoryFrame>(m_height, m_width, m_channels);
            }



        public:
            CameraIntrinsics getCameraIntrinsics() { return std::static_pointer_cast<RGBsharedMemoryFrame>(m_frame)->m_intrinsics; }
            CameraExtrinsics getCameraExtrinsics() { return std::static_pointer_cast<RGBsharedMemoryFrame>(m_frame)->m_extrinsics; }
        };


class CameraReader
        {
        public:
            CameraReader();
            std::shared_ptr<RGBSharedMemoryTool> D435_cam;
            //cam_params params;
            cv::Mat frame;
            cv::Mat Depthframe;
            bool _debug;
            int _cam_type;
            unsigned int getWidth();
            unsigned int getHeight();
            void readFrame();
            cv::Mat getCameraMatrix();
            cv::Mat getDistortion();
            long getTimestamp();
        };
#endif //__ARUCO_READER_SHARED_MEMORY_TOOL_HPP__
