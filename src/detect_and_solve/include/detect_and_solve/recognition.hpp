#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

const float scale = 1;

bool recognition_main(const cv::Mat&originalImg, std::vector<cv::RotatedRect>& out);
bool recognition_number(const cv::Mat&originalImg, std::vector<cv::RotatedRect>& out);
bool refineStrip(const cv::Mat&originalImg, cv::RotatedRect rect, cv::RotatedRect& out);
void debug_frame(cv::Mat Img, bool cvt_to_bgr=true);