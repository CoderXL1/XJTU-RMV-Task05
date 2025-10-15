#pragma once
#include "MvCameraControl.h"
#include <vector>
#include <opencv2/opencv.hpp>

class HikCameraDriver {
public:
    HikCameraDriver();
    ~HikCameraDriver();

    bool openCamera(uint32_t index = 0);
    void closeCamera();
    
    bool setExposure(int value);
    bool setGain(int value);
    bool setFPS(float fps);
    bool setPixelFormat(std::string pixelFormat);
    bool appendCallback(MvImageCallbackEx callback, void* user = nullptr);
    bool startGrabbing();
    bool stopGrabbing();
    bool getFPS(float &fps);
    bool grabFrame(cv::Mat &image);//deprecated, use appendCallback instead
    inline bool Check(int nRet);

private:
    void* handle_; // MVS SDK 相机句柄
};
