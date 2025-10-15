#include "hik_camera/hik_camera_driver.hpp"
#include <iostream>
#include <cstring>             // memset
#include <opencv2/opencv.hpp>

inline bool HikCameraDriver::Check(int nRet)
{
    return nRet == MV_OK;
}

HikCameraDriver::HikCameraDriver() : handle_(nullptr) 
{
    MV_CC_Initialize();
}

HikCameraDriver::~HikCameraDriver() {
    closeCamera();
    MV_CC_Finalize();
}

bool HikCameraDriver::openCamera(uint32_t index) {
    int nRet;
    // unsigned int nDeviceCount = 0;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (!Check(nRet) || stDeviceList.nDeviceNum == 0) {
        std::cerr << "No camera found!" << std::endl;
        return false;
    }

    if (index >= stDeviceList.nDeviceNum) {
        std::cerr << "Camera index out of range!" << std::endl;
        return false;
    }

    // 创建相机实例
    nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[index]);
    if (!Check(nRet)) {
        std::cerr << "Create handle failed!" << std::endl;
        return false;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle_);
    if (!Check(nRet)) {
        std::cerr << "Open camera failed!" << std::endl;
        return false;
    }

    // 设置为连续采集模式
    nRet = MV_CC_SetEnumValue(handle_, "AcquisitionMode", 2);
    if (!Check(nRet)) {
        std::cerr << "Set AcquisitionMode failed!" << std::endl;
    }

    nRet = MV_CC_SetImageNodeNum(handle_, 1);
    if (!Check(nRet)) {
        std::cerr << "Set ImageNodeNum failed!" << std::endl;
        return false;
    }

    std::cout << "Camera opened successfully!" << std::endl;
    return true;
}

void HikCameraDriver::closeCamera() {
    if (handle_) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        // MV_CC_Finalize();
        std::cout << "Camera closed successfully!" << std::endl;
        handle_ = nullptr;
    }
}

/* void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
// {
//     if (pFrameInfo)
//     {
//         printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n", 
//             pFrameInfo->nExtendWidth, pFrameInfo->nExtendHeight, pFrameInfo->nFrameNum);
//     }
// }*/

bool HikCameraDriver::appendCallback(MvImageCallbackEx callback, void* pUser)
{
    if (!handle_) return false;
    int nRet = MV_CC_RegisterImageCallBackEx(handle_, callback, pUser);
    return Check(nRet);
}

bool HikCameraDriver::startGrabbing() {
    if (!handle_) return false;
    int nRet = MV_CC_StartGrabbing(handle_);
    return Check(nRet);
}

bool HikCameraDriver::stopGrabbing() {
    if (!handle_) return false;
    int nRet = MV_CC_StopGrabbing(handle_);
    return Check(nRet);
}

bool HikCameraDriver::setExposure(int value) {
    if (!handle_) return false;
    int nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", value);
    return Check(nRet);
}

bool HikCameraDriver::setGain(int value) {
    if (!handle_) return false;
    int nRet = MV_CC_SetFloatValue(handle_, "Gain", value);
    return Check(nRet);
}

bool HikCameraDriver::setFPS(float fps) {
    if (!handle_) return false;
    int nRet1 = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    int nRet2 = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", fps);
    return Check(nRet1)&&Check(nRet2);
}

bool HikCameraDriver::setPixelFormat(std::string pixelFormat) {
    if (!handle_) return false;
    int nRet = MV_CC_SetEnumValueByString(handle_, "PixelFormat", pixelFormat.c_str());
    return Check(nRet);
}

bool HikCameraDriver::getFPS(float &fps) {
    if (!handle_) return false;
    MVCC_FLOATVALUE FloatValue;
    int nRet = MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &FloatValue);
    if (Check(nRet)) {
        fps = FloatValue.fCurValue;
        return true;
    }
    return false;
}

// int main() {
//     HikCameraDriver driver;
//     if (!driver.openCamera()) return -1;

//     driver.setExposure(5000);
//     driver.setGain(0);
//     driver.setFrameRate(30);

//     cv::Mat frame;
//     if (driver.grabFrame(frame)) {
//         cv::imshow("Frame", frame);
//         cv::waitKey(0);
//     }

//     driver.closeCamera();
//     return 0;
// }
