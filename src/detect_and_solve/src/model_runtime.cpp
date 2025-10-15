#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include <nlohmann/json.hpp> // for digesting classes.json
#include "detect_and_solve/model_runtime.hpp"
#include "detect_and_solve/util.hpp"

using json = nlohmann::json;
//#define DEBUG_MODE

bool ModelDriver::LoadModel(std::string model_folder_)
{
    std::string json_path = model_folder_+"/classes.json";
    std::string model_path = model_folder_+"/tiny_lenet.onnx";
    if(!std::filesystem::exists(json_path)||!std::filesystem::exists(model_path))
        return false;
    // load ONNX model
    net_ = cv::dnn::readNetFromONNX(model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // load class list
    std::ifstream f(json_path);
    json j;
    f >> j;
    for (auto &cls : j) {
        classes_.push_back(cls.get<std::string>());
    }
    negative_class_index_ = classes_.size() - 1;
    return true;
}

#define THRES_V 70  // adjustable threshold: 0~255, the bigger, the whiter it has to be

bool ModelDriver::classify(const cv::Mat roi, std::string& res, double& prob)
{
#ifdef DEBUG_MODE
    debug_frame(roi);
#endif DEBUG_MODE

    // --- convert to gray ---
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
#ifdef DEBUG_MODE
    debug_frame(gray);
#endif DEBUG_MODE

    // --- enhance contrast ---
    // Step 1: 计算标准差判断对比度
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);

    double contrast = stddev[0];
    if (contrast < 40.0)  // 阈值可调，小于 40 说明图像偏灰、对比度低
    {
        // 使用 CLAHE 增强
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        cv::Mat enhanced;
        clahe->apply(gray, enhanced);

        // Gamma 校正（微调亮度层次）
        cv::Mat gamma_corrected;
        enhanced.convertTo(gamma_corrected, CV_32F, 1.0 / 255.0);
        cv::pow(gamma_corrected, 0.9, gamma_corrected); // gamma < 1 增强亮区
        gamma_corrected.convertTo(gray, CV_8U, 255.0);

#ifdef DEBUG_MODE
        debug_frame(gray);
#endif DEBUG_MODE
    }
    // 阈值化：高亮部分置白，其余置黑
    cv::Mat input;
    cv::threshold(gray, input, THRES_V, 255, cv::THRESH_BINARY);

    //important! aspect ratio 2:3
    cv::resize(input, input, cv::Size(32, 48)); // (W,H)
#ifdef DEBUG_MODE
    debug_frame(input);
#endif DEBUG_MODE

    // --- 归一化 ---
    input.convertTo(input, CV_32F, 1.0 / 255);
    input = (input - 0.5) / 0.5; // Normalize (-1,1)

    // 转 blob [1,1,H,W]
    cv::Mat blob = cv::dnn::blobFromImage(input);

    // --- 推理 ---
    net_.setInput(blob);
    cv::Mat out = net_.forward(); // shape [1, num_classes]

    // --- 找最大概率 ---
    cv::Point classIdPoint;
    double maxVal;
    cv::minMaxLoc(out, 0, &maxVal, 0, &classIdPoint);
    int top_class = classIdPoint.x;

    // --- 阈值判断 ---
    const double THRESH = 0.8;
    std::string result;
    if (top_class == negative_class_index_ || maxVal < THRESH) {
        result = "No match";
    } else {
        result = classes_[top_class];
    }

    res = result;
    prob = maxVal;

    return true;
}
