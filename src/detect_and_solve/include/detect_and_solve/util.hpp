#pragma once
#include <opencv2/opencv.hpp>

void plot_rect_red(cv::Mat&img, const cv::RotatedRect&rect);
void plot_rect_green(cv::Mat&img, const cv::RotatedRect&rect);
cv::Point2f normalize(cv::Point2f x);