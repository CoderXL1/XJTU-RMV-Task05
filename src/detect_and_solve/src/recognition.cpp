//
//  recognition.cpp
//  Created by Leo Xia
//  Created with VSCode on Ubuntu 22.04
//

#include "detect_and_solve/recognition.hpp"
using namespace cv;
using namespace std;

Mat originalImg, grayImg, blurImg, hsvImg, medianImg, binaryImg, adaptiveImg;
Mat edgeImg, morphImg, rectImg;
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;

bool angular_vaidation(RotatedRect&rect)
{
    double angle, height, width;
    angle=rect.angle;
    height=rect.size.height;
    width=rect.size.width;
    bool ind=false;
    while(angle>45.0)
    {
        angle-=90.0;
        ind^=1;
    }
    if(!(angle<=7.0&&angle>=-7.0)) return false;
    if(ind)
        return 5<width/height;//&&width/height<7;
    else 
        return 5<height/width;//&&height/width<7;
}

Point2f center_of_mass(const RotatedRect& rect)
{
    Point2f pts[4],ret=Point2f(0.0,0.0);
    rect.points(pts);
    for(auto pt:pts)
    {
        // cout<<pt<<"\n";
        ret+=pt/4;
    }
    return ret;
}
void debug_frame(Mat Img)
{
    double scale = 0.5; // 缩小到 50%
    cv::Mat small;
    cv::resize(Img, small, cv::Size(), scale, scale);
    cv::imshow("Img", small);
    cv::waitKey(0);
}
bool recognition_main(const Mat&originalImg, RotatedRect& rect1, RotatedRect& rect2)
{
    if(originalImg.empty())
    {
        cout<<"Failed to load image!"<<endl;
        return -1;
    }
    cv::namedWindow("Img", cv::WINDOW_NORMAL);
    // debug_frame(originalImg);

    cvtColor(originalImg, hsvImg, COLOR_BGR2HSV);
    cv::Scalar lower_blue(70, 0, 200); // H,S,V下限
    cv::Scalar upper_blue(130, 255, 255); // H,S,V上限
    // cv::Mat mask;
    cv::inRange(hsvImg, lower_blue, upper_blue, binaryImg);
    // cvtColor(guassianImg, grayImg, COLOR_BGR2GRAY);
    // threshold(grayImg, binaryImg, 192, 255, THRESH_BINARY);
    // debug_frame(grayImg);
    morphologyEx(binaryImg, morphImg, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(10, 10)));
    binaryImg = morphImg.clone();
    morphologyEx(binaryImg, morphImg, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(10, 10)));
    Canny(morphImg,edgeImg, 100, 200);
    findContours(edgeImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // debug_frame(edgeImg);
    // rectImg=originalImg.clone();
    // marking the current rect: rect1 or rect2
    int curr=0;
    for(size_t i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i]) < 1000) continue;
        RotatedRect rect = minAreaRect(contours[i]);
        // cout<<rect.angle<<" "<<rect.size.height<<" "<<rect.size.width<<endl;
        if(true||angular_vaidation(rect))
        {
            cout<<" ok"<<endl;
            if(!curr)
            {
                rect1=rect;
                curr=1;
            }
            else
            {
                rect2=rect;
                curr=2;
                break;
            }
        }
        // else cout<<" bad"<<endl;
    }
    if(curr<2)return false;
    if(center_of_mass(rect1).x>center_of_mass(rect2).x)
        swap(rect1,rect2);
    return true;
}
