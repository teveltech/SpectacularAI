//
// Created by gal on 10/24/21.
//

#include "sharedmemtool.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include "farConfigParser.hpp"

sharedMemoryTool::sharedMemoryTool()
:  m_frame(nullptr)
, m_smFileName()
, m_smName()
, m_smBuffer(nullptr)
, m_smBufferSize(0)
, m_width(0)
, m_height(0)
, m_memoryCont(nullptr)
{}

sharedMemoryTool::~sharedMemoryTool()
{
    if(m_smBuffer)
    {
        delete[] m_smBuffer;
    }
}

bool sharedMemoryTool::initSM()
{
    auto config = farConfigParser(utilities::globals::SYSTEM_CONFIG_FILE_NAME);
    auto cameraDevices = config.getCameraFrontEnd()->getAllDevices();
    std::string deviceJsonFileName;
    for (auto it = cameraDevices.begin(); it != cameraDevices.end(); ++it)
    {
        if(it->find(m_deviceName) != std::string::npos)
        {
            auto cfg = config.getCameraFrontEnd()->getDeviceConfig(*it);
            deviceJsonFileName =cfg->getName() + ".json";
            auto res = std::static_pointer_cast<cameraConfig>(cfg)->initValueResolution();
            m_width = res.first;
            m_height = res.second;
            createFrameSharedPointer();
            break;
        }
    }
    if (!deviceJsonFileName.empty())
    {
        m_smFileName = deviceJsonFileName;
    }
    else
    {
        utilities::loggerUtility::writeLog(TVL_LOG_ERROR, "sharedMemoryTool::initSM(), DEVICE NOT FOUND, %s", m_deviceName.c_str());
        return false;
    }

    m_smName = m_smFileName.substr(0, m_smFileName.find(".json"));
    m_memoryCont = std::make_shared<sharedMemoryContainer>(m_smFileName);
    while(!utilities::mainframe::endOfWorldAnnouncer::isEndOfWorldArrive() && !m_memoryCont->isInitSuccessfully())
    {
        utilities::loggerUtility::writeLog(TVL_LOG_WARN, "sharedMemoryTool::initSM(), WAITING FOR UVC DEVICE TO START");
        utilities::timeUtility::sleepMilli(utilities::timeUtility::MILLIS_T0_SECOND * 2);
        m_memoryCont.reset();
        m_memoryCont = std::make_shared<sharedMemoryContainer>(m_smFileName);
    }
    m_memoryData = m_memoryCont->getSmData()[m_smName];
    m_smBufferSize = m_memoryData.m_size;
    m_smBuffer = new char[m_smBufferSize];
    loadNewFrame();
    while(!utilities::mainframe::endOfWorldAnnouncer::isEndOfWorldArrive() && utilities::timeUtility::getCurrentMicro() - m_frame->m_timeStamp > utilities::timeUtility::MICRO_T0_SECOND)
    {
        utilities::loggerUtility::writeLog(TVL_LOG_WARN, "sharedMemoryTool::initSM(), WAITING FOR UVC DEVICE FIRST UPDATE SHARED MEMORY");
        utilities::timeUtility::sleepMilli(utilities::timeUtility::MILLIS_T0_SECOND * 2);
        m_memoryCont.reset();
        m_memoryCont = std::make_shared<sharedMemoryContainer>(m_smFileName);
        if(m_memoryCont->isInitSuccessfully())
        {
            loadNewFrame();
        }
    }
    return (!utilities::mainframe::endOfWorldAnnouncer::isEndOfWorldArrive());
}

bool sharedMemoryTool::loadNewFrame()
{
    static long timeStamp;;
    auto ret = m_memoryCont->readFromSM(m_smName, (void**)&m_smBuffer, m_smBufferSize);
    if(ret == m_memoryData.m_size)
    {
        memcpy((&timeStamp), (m_smBuffer+m_memoryData.m_data["SHARED_MEM_TIMESTAMP"].m_offset), m_memoryData.m_data["SHARED_MEM_TIMESTAMP"].m_length);
        if(timeStamp != m_frame->m_timeStamp)
        {
            m_frame->m_timeStamp = timeStamp;
            memcpy((m_frame->m_frame), (m_smBuffer+m_memoryData.m_data[m_frameNameInSharedMemory].m_offset), m_memoryData.m_data[m_frameNameInSharedMemory].m_length);
            memcpy((&m_frame->m_fps), (m_smBuffer+m_memoryData.m_data["FPS"].m_offset), m_memoryData.m_data["FPS"].m_length);
            m_cvMatFrame = cv::Mat(m_frame->m_height, m_frame->m_width, m_cvType, m_frame->m_frame);
            loadAddedDataFromSharedMemory();
        }
        return true;
    }
    else
    {
        utilities::loggerUtility::writeLog(TVL_LOG_ERROR, "sharedMemoryTool::loadNewFrame(), PROBLEM READING DATA FROM SHARED MEMORY");
        return false;
    }
}

void RGBSharedMemoryTool::loadAddedDataFromSharedMemory()
{
    std::static_pointer_cast<RGBsharedMemoryFrame>(m_frame)->m_intrinsics.deserialize(m_smBuffer+m_memoryData.m_data["RGB_CAMERA_INTRINSICS"].m_offset);
    std::static_pointer_cast<RGBsharedMemoryFrame>(m_frame)->m_extrinsics.deserialize(m_smBuffer+m_memoryData.m_data["CAMERA_EXTRINSICS"].m_offset);
}

void sharedMemoryTool::getFrameAsCvMat(cv::Mat &frame)
{
    auto now = utilities::timeUtility::getCurrentMicro();

    if(m_cvMatFrame.empty() || lastTimeStampRead == this->getTimestamp())
    {
        if (now - lastPrintNoFrameTime >= utilities::timeUtility::MICRO_T0_SECOND * 5 and m_cvMatFrame.empty())
            // after first alert, keep sending each 5 seconds
            {
            lastPrintNoFrameTime = now;
            utilities::loggerUtility::writeLog(TVL_LOG_DEBUG, "aruco_reader::routine(), ArucoReader"
                                                              " Received empty frame");
            }
    } else{
        lastTimeStampRead = this->getTimestamp();
        m_cvMatFrame.copyTo(frame);
    }
}


RGBSharedMemoryTool::RGBSharedMemoryTool()
: sharedMemoryTool()
{
    m_channels = 3;
    m_deviceName = "FRONT_CAMERA";
    m_frameNameInSharedMemory = "RGB";
    m_cvType = CV_8UC3;
    if(initSM())
    {
        utilities::loggerUtility::writeLog(TVL_LOG_INFO, "RGBSharedMemoryTool::RGBSharedMemoryTool(), INIT SUCCESSFULLY");
    }
    else
    {
        utilities::loggerUtility::writeLog(TVL_LOG_ERROR, "RGBSharedMemoryTool::RGBSharedMemoryTool(), DID NOT INIT SUCCESSFULLY");
    }
}

CameraReader::CameraReader()
{
        D435_cam = std::make_shared<RGBSharedMemoryTool>();
        if(D435_cam->loadNewFrame())
        {
            D435_cam->getFrameAsCvMat(frame);
            params.init(D435_cam->getCameraIntrinsics());
        }
}

void CameraReader::readFrame()
{
        if(D435_cam->loadNewFrame())
        {
            D435_cam->getFrameAsCvMat(frame);
            try
            {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            }
            catch (std::exception& ex)
            {
                std::cout << "Exception :" << ex.what() << std::endl;
            }
        }
}

unsigned int CameraReader::getWidth()
{
    return  D435_cam->getWidth();
}

unsigned int CameraReader::getHeight()
{
    return D435_cam->getHeight();
}

cv::Mat CameraReader::getCameraMatrix()
{
    return params.CameraMatrix;
}

cv::Mat CameraReader::getDistortion()
{
    return params.Distorsion;
}

long CameraReader::getTimestamp()
{
    return D435_cam->getTimestamp();
}



