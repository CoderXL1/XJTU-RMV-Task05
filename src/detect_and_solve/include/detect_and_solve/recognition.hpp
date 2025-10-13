#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
bool recognition_main(const cv::Mat&originalImg, std::vector<cv::RotatedRect>& out);
bool recognition_number(const cv::Mat&originalImg, std::vector<cv::RotatedRect>& out);
void debug_frame(cv::Mat Img);