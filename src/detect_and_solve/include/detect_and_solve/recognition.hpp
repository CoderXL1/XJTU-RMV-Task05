#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

bool recognition_main(const cv::Mat&originalImg, cv::RotatedRect& rect1, cv::RotatedRect& rect2);
void debug_frame(cv::Mat Img);