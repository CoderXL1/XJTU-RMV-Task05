//
//  recognition.cpp
//  Created by Leo Xia
//  Created with VSCode on Ubuntu 22.04
//

#include <algorithm>
#include "detect_and_solve/recognition.hpp"
using namespace cv;
using namespace std;

extern const float scale;

Mat grayImg, blurImg, hsvImg, medianImg, binaryImg;
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

void debug_frame(Mat Img, bool cvt_to_bgr)
{
    double wscale = 0.5; // 缩小到 50%
    cv::Mat small;
    cv::resize(Img, small, cv::Size(), wscale, wscale);
    if(cvt_to_bgr&&small.channels()!=1)
        cv::cvtColor(small,small,COLOR_HSV2BGR);
    cv::imshow("Img", small);
    cv::waitKey(0);
}
bool cmp(const RotatedRect&a, const RotatedRect&b)
{
    return a.center.x<b.center.x;
}
bool recognition_main(const Mat&hsvImg, vector<RotatedRect>& out)
{
    // cv::namedWindow("Img", cv::WINDOW_NORMAL);
    // debug_frame(originalImg);

    // color space is now converted in node.cpp
    // cvtColor(originalImg, hsvImg, COLOR_BGR2HSV);
    cv::Mat mask1,mask2;
    cv::inRange(hsvImg, cv::Scalar(90, 130, 150), cv::Scalar(110, 200, 255), mask1);
    // debug_frame(mask1);
    cv::inRange(hsvImg, cv::Scalar(0, 0, 254), cv::Scalar(180, 255, 255), mask2);
    // debug_frame(mask2);
    binaryImg = mask1+mask2;
    morphologyEx(binaryImg, morphImg, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size((int)10*scale, (int)10*scale)));
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
        if(contourArea(contours[i]) < 500*scale*scale) continue;
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

bool recognition_number(const cv::Mat&hsvImg, std::vector<cv::RotatedRect>& out)
{
    // cvtColor(originalImg, hsvImg, COLOR_BGR2HSV);
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
        if(rect.size.area()<10000*scale*scale)continue;
        // if(rect.size.area()>1000000*scale*scale)continue;
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

cv::Vec4f fitLineToContour(const std::vector<cv::Point>& contour)
{
    cv::Vec4f line;
    cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

    float vx = line[0];
    float vy = line[1];
    float x0 = line[2];
    float y0 = line[3];

    // 将轮廓点在方向向量上的投影求最小值和最大值
    float minProj = FLT_MAX, maxProj = -FLT_MAX;
    for (const auto& p : contour)
    {
        float proj = (p.x - x0) * vx + (p.y - y0) * vy;
        minProj = std::min(minProj, proj);
        maxProj = std::max(maxProj, proj);
    }

    // 计算对应的端点坐标
    cv::Point2f p1(x0 + vx * minProj, y0 + vy * minProj);
    cv::Point2f p2(x0 + vx * maxProj, y0 + vy * maxProj);

    return cv::Vec4f(p1.x, p1.y, p2.x, p2.y);  // 这是真实的线段端点
}

bool refineStrip(const cv::Mat&hsvImg, cv::RotatedRect rect, cv::RotatedRect&out)
{
    cv::Rect roi_box = rect.boundingRect();
    roi_box &= cv::Rect(0, 0, hsvImg.cols, hsvImg.rows); // prevent overflow
    cv::Mat roi = hsvImg(roi_box);//.clone();
    // debug_frame(roi);
    Mat roi_channels[3];
    split(roi, roi_channels);
    Canny(roi_channels[2],edgeImg,100, 200);
    // debug_frame(edgeImg);
    // cv::Scalar lower_sat(75, 0, 150); // H,S,V下限
    // cv::Scalar upper_sat(95, 255, 255); // H,S,V上限
    // cv::inRange(roi, lower_sat, upper_sat, binaryImg);
    // debug_frame(binaryImg);
    // std::vector<std::vector<cv::Point>> contours;
    contours.clear();
    cv::findContours(edgeImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> all_points;
    for (const auto& c : contours)
    {
        all_points.insert(all_points.end(), c.begin(), c.end());
    }
    // 拟合直线
    if(all_points.empty())return false;
    out=cv::minAreaRect(all_points);
    out.center.x += roi_box.x;
    out.center.y += roi_box.y;
    return true;
}