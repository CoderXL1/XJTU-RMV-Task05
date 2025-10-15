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

void debug_frame(cv::Mat Img, bool cvt_to_bgr)
{
    double wscale = 0.5; // 缩小到 50%
    cv::Mat small;
    cv::resize(Img, small, cv::Size(), wscale, wscale);
    if(cvt_to_bgr&&small.channels()!=1)
        cv::cvtColor(small,small,cv::COLOR_HSV2BGR);
    cv::imshow("Img", small);
    cv::waitKey(0);
}