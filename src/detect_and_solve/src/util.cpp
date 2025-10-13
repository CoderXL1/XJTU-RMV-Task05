#include <opencv2/opencv.hpp>
#include "detect_and_solve/util.hpp"

void plot_rect_red(cv::Mat&img, const cv::RotatedRect&rect)
{
    cv::Point2f corners[4];
    rect.points(corners);
    for(int i=0;i<4;i++)
    {
        cv::line(img, corners[i], corners[(i+1)%4],cv::Scalar(0,0,255));
    }
}
void plot_rect_green(cv::Mat&img, const cv::RotatedRect&rect)
{
    cv::Point2f corners[4];
    rect.points(corners);
    for(int i=0;i<4;i++)
    {
        cv::line(img, corners[i], corners[(i+1)%4],cv::Scalar(0,255,0));
    }
}

cv::Point2f normalize(cv::Point2f x)
{
    return x/cv::norm(x);
}