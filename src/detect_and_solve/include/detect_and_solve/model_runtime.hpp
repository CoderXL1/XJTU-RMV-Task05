#pragma once
#include <string>

class ModelDriver
{
public:
    bool LoadModel(std::string model_folder_);
    bool classify(cv::Mat roi, std::string&res, double&prob);
private:
    cv::dnn::Net net_;
    std::vector<std::string> classes_;
    int negative_class_index_;
};