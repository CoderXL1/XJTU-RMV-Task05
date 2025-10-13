//
//  recognition.cpp
//  Created by Leo Xia
//  Created with VSCode on Ubuntu 22.04
//

#include <algorithm>
#include "detect_and_solve/recognition.hpp"
using namespace cv;
using namespace std;

Mat originalImg, grayImg, blurImg, hsvImg, medianImg, binaryImg, adaptiveImg;
Mat edgeImg, morphImg, rectImg;
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;

bool angular_vaidation(RotatedRect&rect)
{
    return abs(rect.angle)<10;

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

void debug_frame(Mat Img)
{
    double scale = 0.5; // 缩小到 50%
    cv::Mat small;
    cv::resize(Img, small, cv::Size(), scale, scale);
    cv::imshow("Img", small);
    cv::waitKey(0);
}
bool cmp(const RotatedRect&a, const RotatedRect&b)
{
    return a.center.x<b.center.x;
}
bool recognition_main(const Mat&originalImg, vector<RotatedRect>& out)
{
    // cv::namedWindow("Img", cv::WINDOW_NORMAL);
    // debug_frame(originalImg);

    cvtColor(originalImg, hsvImg, COLOR_BGR2HSV);
    cv::Mat mask1,mask2;
    cv::inRange(hsvImg, cv::Scalar(90, 80, 80), cv::Scalar(115, 255, 255), mask1);
    // debug_frame(mask1);
    cv::inRange(hsvImg, cv::Scalar(0, 0, 120), cv::Scalar(180, 255, 255), mask2);
    // debug_frame(mask2);
    binaryImg = mask1+mask2;
    morphologyEx(binaryImg, morphImg, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(10, 10)));
    // debug_frame(morphImg);
    binaryImg=morphImg.clone();
    morphologyEx(binaryImg, morphImg, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));
    // binaryImg = morphImg.clone();
    // debug_frame(morphImg);
    // Canny(morphImg,edgeImg, 100, 200);
    findContours(morphImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // debug_frame(edgeImg);
    // rectImg=originalImg.clone();
    // marking the current rect: rect1 or rect2
    vector<RotatedRect> rects;
    for(size_t i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i]) < 500) continue;
        RotatedRect rect = minAreaRect(contours[i]);
        // Point2f corners[4];
        // rect.points(corners);
        // for(int i=0;i<4;i++)
        // {
        //     line(originalImg, corners[i], corners[(i+1)%4],Scalar(0,255,0));
        // }
        // imshow("plot",originalImg);
        // waitKey(0);
        // cout<<rect.angle<<" "<<rect.size.height<<" "<<rect.size.width<<endl;
        if(true||angular_vaidation(rect))
        {
            cout<<" ok";
            rects.push_back(rect);
        }
        // else cout<<" bad"<<endl;
    }
    cout<<endl;
    if(rects.size()<2)return false;
    sort(rects.begin(),rects.end(),cmp);
    /*double best_err=10000000.0;
    // int curr_i=0,curr_j=0;
    // for(int i=0,j=1;i<rects.size()&&j<rects.size();)
    // {
    //     double err=real_dist-cv::norm(rects[i].center-rects[j].center);
    //     if(abs(err)<best_err)
    //     {
    //         curr_i=i;
    //         curr_j=j;
    //         best_err=abs(err);
    //     }
    //     if(err<0)i++;
    //     else j++;
    //     if(i>=j)j++;
    // }
    vector<Point2f> rect1_p(4),rect2_p(4),all_points;
    rects[curr_i].points(rect1_p.data());
    rects[curr_j].points(rect2_p.data());
    all_points.insert(all_points.end(), rect1_p.begin(), rect1_p.end());
    all_points.insert(all_points.end(), rect2_p.begin(), rect2_p.end());
    out=cv::minAreaRect(all_points);*/
    out=rects;
    return true;
}

bool recognition_number(const cv::Mat&originalImg, std::vector<cv::RotatedRect>& out)
{
    cvtColor(originalImg, hsvImg, COLOR_BGR2HSV);
    cv::Scalar lower_sat(0, 0, 50); // H,S,V下限
    cv::Scalar upper_sat(180, 140, 200); // H,S,V上限
    cv::inRange(hsvImg, lower_sat, upper_sat, binaryImg);
    // imshow("img",binaryImg);
    // waitKey(0);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(binaryImg, morphImg, MORPH_OPEN, kernel);
    binaryImg=morphImg.clone();
    morphologyEx(binaryImg, morphImg, MORPH_CLOSE, kernel);
    Canny(morphImg, edgeImg, 100, 200);
    findContours(edgeImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<RotatedRect> rects;
    for(size_t i = 0; i< contours.size();i++)
    {
        RotatedRect rect = minAreaRect(contours[i]);
        if(rect.size.area()<10000)continue;
        if(rect.size.area()>100000)continue;
        rects.push_back(rect);
        cout<<" np";
        // Point2f corners[4];
        // rect.points(corners);
        // for(int i=0;i<4;i++)
        // {
        //     line(originalImg, corners[i], corners[(i+1)%4],Scalar(0,255,0));
        // }
        // imshow("plot",originalImg);
        // waitKey(0);
    }
    cout<<endl;
    out = rects;
    return true;
}