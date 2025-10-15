#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "detect_and_solve/recognition.hpp"
#include "detect_and_solve/util.hpp"

extern const float scale;

// #define DEBUG_MODE

namespace fs=std::filesystem;

class DetectorAndSolver : public rclcpp::Node
{
public:
  DetectorAndSolver():Node("detect_and_solve")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing detect_and_solve node...");
    // params: path for json, size of light strip(mm), use_video option, video path or topic path
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("detect_and_solve");

    this->declare_parameter<std::string>("camera_json", package_share_dir +"/config/camera.json");
    // this->declare_parameter<double>("width_mm", 140.0);
    // this->declare_parameter<double>("height_mm", 55.0); 
    declare_parameter<bool>("use_video", false);
    declare_parameter<std::string>("path", "/camera/image_raw");
    
    camera_json_ = this->get_parameter("camera_json").as_string();
    std::cout<<camera_json_<<std::endl;//print!
    // rect_w_ = this->get_parameter("width_mm").as_double();
    // rect_h_ = this->get_parameter("height_mm").as_double();
    use_video=this->get_parameter("use_video").as_bool();
    path_ = this->get_parameter("path").as_string();
    if (!loadCameraParams(camera_json_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load intrinsics and distortion parameters from: %s", camera_json_.c_str());
      throw std::runtime_error("config load failure");
    }
    //Hard Coding Intrinsic Matrix
    /*
    // cameraMatrix_ = (cv::Mat_<double>(3,3) << 
    //   2584.809123, 0.0, 1540.358948,
    //   0.0, 2570.871247, 1087.701687,
    //   0.0, 0.0, 1.0
    // );

    // distCoeffs_ = (cv::Mat_<double>(1,4) << 
    //     -0.074792, 0.105821, 0.0, 0.0
    // );
    */
    
    // Subscriber
    if (use_video) {
      capture_.open(path_);  // from file
      if (!capture_.isOpened()) {
          RCLCPP_ERROR(get_logger(), "Failed to open video file: %s", path_.c_str());
          rclcpp::shutdown();
          return;
      }
    } else {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          path_, 10, std::bind(&DetectorAndSolver::imageCallback, this, std::placeholders::_1));
    }
    detector_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/image_marked", 10);
    np_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/number_pattern", 10);
    // Publisher for pose (optional, useful)
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detector/pose", 10);

    RCLCPP_INFO(this->get_logger(), "detect_and_solve node started. camera_json: %s, rect: %.2f x %.2f mm",
                camera_json_.c_str(), rect_w_, rect_h_);

    if(use_video)
      videoProcess();
  }
  private:
  // Parameters
  std::string camera_json_;
  double rect_w_, rect_h_;
  std::string path_;
  bool use_video;

  // OpenCV camera params
  cv::Mat cameraMatrix_, distCoeffs_;
  cv::Mat newCameraMatrix_; // optional optimized intrinsics

  // OpenCV video capture
  cv::VideoCapture capture_;

  // ROS subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  // ROS publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detector_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr np_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  bool loadCameraParams(const std::string &json_path) {
    if (!fs::exists(json_path)) {
      RCLCPP_ERROR(this->get_logger(), "camera json does not exist: %s", json_path.c_str());
      return false;
    }

    cv::FileStorage fs(json_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "failed to open json: %s", json_path.c_str());
      return false;
    }

    // camera_matrix is named "camera_matrix"
    if (fs["camera_matrix"].isNone()) {
      RCLCPP_ERROR(this->get_logger(), "camera_matrix not found in json");
      return false;
    }
    fs["camera_matrix"] >> cameraMatrix_;

    cameraMatrix_.at<double>(0, 0) *= scale;
    cameraMatrix_.at<double>(1, 1) *= scale;
    cameraMatrix_.at<double>(0, 2) *= scale;
    cameraMatrix_.at<double>(1, 2) *= scale;

    // distortion is named "distortion_coefficients"
    if (!fs["distortion_coefficients"].isNone()) {
      fs["distortion_coefficients"] >> distCoeffs_;
    } else {
      RCLCPP_WARN(this->get_logger(), "no distortion key found - assume zero");
      distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }

    if (!fs["board_size"].isNone()&&!fs["board_size"]["width_mm"].isNone()&&!!fs["board_size"]["height_mm"].isNone()) {
      rect_w_=(double)fs["board_size"]["width_mm"];
      rect_h_=(double)fs["board_size"]["height_mm"];
    } else {
      RCLCPP_WARN(this->get_logger(), "no board size specified in json - using default 60.0 x 100.0");
      rect_w_=60.0;
      rect_h_=100.0;
    }

    // Optional: compute optimal new camera matrix for undistort
    // default alpha=0 -> crop, alpha=1 -> keep all pixels.
    // We'll compute but keep original as well.
    // Need an image size later; compute when first frame arrives.

    RCLCPP_INFO(this->get_logger(), "Loaded camera matrix:\n%dx%d", cameraMatrix_.rows, cameraMatrix_.cols);
    RCLCPP_INFO(this->get_logger(), "Dist coeffs size: %dx%d", distCoeffs_.rows, distCoeffs_.cols);
    fs.release();
    return true;
  }

  bool solve_pnp(cv::RotatedRect& rect)//, cv::Mat undistorted)
  {
    const double&w = rect_w_;
    const double&h = rect_h_;
    std::vector<cv::Point2f> rect_corners(4);
    rect.points(rect_corners.data());
    // for(auto i:rect_corners)
    //   std::cout<<i.x<<" "<<i.y<<std::endl;

    // Define rectangle in its local object frame, e.g. z=0 plane:
    // top-left, top-right, bottom-right, bottom-left
    std::vector<cv::Point3f> obj_pts = {
        {-w/2, -h/2, 0},
        { w/2, -h/2, 0},
        { w/2,  h/2, 0},
        {-w/2,  h/2, 0}
    };

    // Solve PnP
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(obj_pts, rect_corners, newCameraMatrix_, cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "solvePnP failed");
      return false;
    }

    // tvec is the translation vector: coordinates of the object origin (top-left corner) in camera frame (units mm)
    RCLCPP_INFO(this->get_logger(),
      "PnP success. tvec (mm) = [%.2f, %.2f, %.2f], rvec = [%.4f, %.4f, %.4f]",
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
      rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));

    // Log rotation matrix
    /*
    RCLCPP_DEBUG(this->get_logger(), "Rotation matrix:\n[%f %f %f; %f %f %f; %f %f %f]",
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));*/

    // Publish PoseStamped (convert mm -> meters)
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "camera_link"; // set appropriately for your TF tree

    // translation: convert to meters
    pose_msg.pose.position.x = tvec.at<double>(0) / 1000.0;
    pose_msg.pose.position.y = tvec.at<double>(1) / 1000.0;
    pose_msg.pose.position.z = tvec.at<double>(2) / 1000.0;

    // convert rotation matrix to quaternion
    // build tf-like quaternion from R (row-major)

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    tf2::Matrix3x3 tf_R(
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    tf2::Quaternion q;
    tf_R.getRotation(q);

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);
    /*
    // Optionally draw result and show (if running with GUI)
    // draw corners and axis for visualization
    // std::vector<cv::Point2f> axis2d;
    // // Project 3D axis points to image for visualization (optional)
    // std::vector<cv::Point3f> axis3d = {
    //   cv::Point3f(0,0,0),
    //   cv::Point3f((float)rect_w_, 0, 0),
    //   cv::Point3f(0, (float)rect_h_, 0),
    //   cv::Point3f(0,0,-(float)rect_w_) // some depth vector
    // };
    // cv::projectPoints(axis3d, rvec, tvec, newCameraMatrix_, cv::Mat(), axis2d);

    // for (size_t i=0;i<axis2d.size();++i) {
    //   cv::circle(undistorted, axis2d[i], 3, cv::Scalar(0,255,0), -1);
    // }
    // for (auto &p: rect_corners) {
    //   cv::circle(undistorted, p, 4, cv::Scalar(0,0,255), -1);
    // }
    // show in window (only if you have DISPLAY)
    // debug_frame(undistorted);
    */
  }

  void processFrame(cv::Mat originalframe)
  {
    // frame should already be BGR
    // Undistort input frame (optional but recommended)
    cv::Mat undistorted, undistorted_hsv, frame, gray, normImg;
    // Scale down to optimize efficiency
    cv::resize(originalframe, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
    if (!newCameraMatrix_.empty()) {
      // if newCameraMatrix_ already computed, use it
      cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_, newCameraMatrix_);
    } else {
      // compute newCameraMatrix for this image size on first call
      newCameraMatrix_ = cv::getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, frame.size(), 0);
      cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_, newCameraMatrix_);
    }

/*    cv::cvtColor(undistorted, gray, cv::COLOR_BGR2GRAY);

    // 局部对比度增强（光照均衡）
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8));
    clahe->apply(gray, normImg);

    // cv::imshow("undistorted",undistorted);
    // Call your recognition routine to extract rectangle corners
    cv::cvtColor(undistorted, undistorted_hsv, cv::COLOR_BGR2HSV);
    cv::Mat channels[3];
    cv::split(undistorted_hsv, channels);
    cv::addWeighted(channels[2], 0.5, normImg, 0.5, 0, channels[2]);
    cv::merge(channels, undistorted_hsv);*/
    cv::cvtColor(undistorted, undistorted_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::RotatedRect> light_strips, number_patterns;
    bool found;
    found = recognition_main(undistorted_hsv, light_strips);
    if (!found) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "recognition_main did not find target in this frame");
      return;
    }
    found = recognition_number(undistorted_hsv, number_patterns);
    if (!found) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "recognition_number did not find target in this frame");
      return;
    }
#ifdef DEBUG_MODE
    cv::Mat plot1=undistorted.clone();
    for(auto rect:light_strips)
    {
      plot_rect_green(plot1, rect);
    }
    for(auto rect:number_patterns)
    {
      plot_rect_red(plot1, rect);
    }
    cv::imshow("plot", plot1);
    cv::waitKey(0);
#endif
    cv::RotatedRect *pattern1,*strip1, *strip2;
    pattern1=nullptr;
    // std::cout<<number_patterns.size()<<std::endl;
#ifdef DEBUG_MODE
    cv::Mat currplot;
#endif
    for(auto &pattern: number_patterns)
    {
      // Find out the 2 nearest but not overlapping strips around this pattern
#ifdef DEBUG_MODE
      currplot = undistorted.clone();
      plot_rect_red(currplot, pattern);
      cv::imshow("plot", currplot);
      cv::waitKey(0);
#endif
      double dist1,dist2;
      strip1=strip2=nullptr;
      dist1=dist2=100000000.0;
      for(auto &strip: light_strips)
      {
        //if not optimal, pass
        // std::cout<<strip.center.x<<" "<<strip.center.y<<std::endl;
        if(cv::norm(pattern.center-strip.center)>=dist2)
        {
#ifdef DEBUG_MODE
          std::cout<<"pass at 292"<<std::endl;
#endif
          continue;
        }
        //if not right ratio, pass
        if(strip.size.aspectRatio()<3&&strip.size.aspectRatio()>0.33)
        {
#ifdef DEBUG_MODE
          std::cout<<"pass at 294"<<std::endl;
#endif
          continue;
        }
        // convert to local frame
        cv::Point2f local = strip.center - pattern.center;

        // calc angle and rotation stuff
        float theta = (float)(-pattern.angle * CV_PI / 180.0);  // inverse rotation
        float cosA = std::cos(theta);
        float sinA = std::sin(theta);

        // rotate the point
        float xRot = local.x * cosA - local.y * sinA;
        float yRot = local.x * sinA + local.y * cosA;

        // Check if it's inside the boundary
        float hw = pattern.size.width  * 0.5f;
        float hh = pattern.size.height * 0.5f;
        if(std::abs(xRot) <= hw && std::abs(yRot) <= hh)
        {
#ifdef DEBUG_MODE
          std::cout<<"pass at 313"<<std::endl;
#endif
          continue;
        }
        dist2 = cv::norm(pattern.center-strip.center);
        strip2=&strip;
        if(dist2<dist1)
        {
          std::swap(dist1,dist2);
          strip2=strip1;
          strip1=&strip;
        }
#ifdef DEBUG_MODE
        cv::Mat currplot2=currplot.clone();
        plot_rect_green(currplot2, strip);
        cv::imshow("plot",currplot2);
        cv::waitKey(0);
#endif
      }
      if(!strip1||!strip2)
      {
#ifdef DEBUG_MODE
        std::cout<<"pass at 323"<<std::endl;
#endif
        continue;
      }
#ifdef DEBUG_MODE
      plot_rect_green(currplot, *strip1);
      plot_rect_green(currplot, *strip2);
      std::cout<<"result:\n";
      std::cout<<pattern.center.x<<" "<<pattern.center.y<<std::endl;
      std::cout<<strip1->center.x<<" "<<strip1->center.y<<std::endl;
      std::cout<<strip2->center.x<<" "<<strip2->center.y<<std::endl;
#endif

      //Check if this potential number pattern lies in the very middle of the 2 strips
#define MAX_ERR 50*scale
      if(cv::norm(pattern.center-(strip1->center+strip2->center)*0.5)>MAX_ERR)
      {
#ifdef DEBUG_MODE
        std::cout<<"pass at 326"<<std::endl;
#endif
        continue;
      }
#undef MAX_ERR
      //Check if these 2 nearest strips are parallel with each other.
      cv::Point2f vtx1[4],vtx2[4];
      strip1->points(vtx1);
      strip2->points(vtx2);
      cv::Point2f h1,h2;//as vector
      if(cv::norm(vtx1[0]-vtx1[1])<cv::norm(vtx1[1]-vtx1[2]))
        h1=normalize(vtx1[1]-vtx1[2]);// /cv::norm(vtx1[1]-vtx1[2]);
      else
        h1=normalize(vtx1[0]-vtx1[1]);

      if(cv::norm(vtx2[0]-vtx2[1])<cv::norm(vtx2[1]-vtx2[2]))
        h2=normalize(vtx2[1]-vtx2[2]);
      else
        h2=normalize(vtx2[0]-vtx2[1]);
#define MAX_ERR 0.26  //about sin(15 deg)
      if(abs(h1.cross(h2))>MAX_ERR)
      {
#ifdef DEBUG_MODE
        std::cout<<"pass at 343"<<std::endl;
#endif
        continue;
      }
#undef MAX_ERR
      pattern1=&pattern;
      break;
    }

    if(!pattern1||!strip1||!strip2)    //Recognition Failed
      return;

#ifdef DEBUG_MODE
    std::cout<<"success!"<<std::endl;
#endif

    //publish number pattern image
    cv::Rect roi_box = pattern1->boundingRect();
    roi_box &= cv::Rect(0, 0, undistorted.cols, undistorted.rows); // prevent overflow
    cv::Mat roi = undistorted(roi_box).clone();
    std_msgs::msg::Header np_header;//np for number_pattern
    np_header.stamp = this->now();
    np_header.frame_id = "camera_link";
    auto img_msg1 = cv_bridge::CvImage(np_header, "bgr8", roi).toImageMsg();
    np_pub_->publish(*img_msg1);

    cv::RotatedRect strip_ref1,strip_ref2;// ref for refined
    refineStrip(undistorted_hsv, *strip1, strip_ref1);
    refineStrip(undistorted_hsv, *strip2, strip_ref2);
    plot_rect_green(undistorted, strip_ref1);
    plot_rect_green(undistorted, strip_ref2);
#ifdef DEBUG_MODE
    // debug_frame(undistorted, false);
#endif
    plot_rect_red(undistorted, *pattern1);
    
#ifdef DEBUG_MODE
    cv::imshow("plot",undistorted);
    cv::waitKey(0);
#endif
    //publish marked image
    std_msgs::msg::Header mk_header;//mk for marked
    mk_header.stamp = this->now();
    mk_header.frame_id = "camera_link";
    auto img_msg2 = cv_bridge::CvImage(mk_header, "bgr8", undistorted).toImageMsg();
    detector_pub_->publish(*img_msg2);

    //solve pnp
    cv::Point2f strip_pts1[4], strip_pts2[4];
    std::vector<cv::Point2f> strip_pts(8);
    strip_ref1.points(strip_pts1);
    strip_ref2.points(strip_pts2);
    for(int i=0;i<4;i++)
    {
      strip_pts.push_back(strip_pts1[i]);
      strip_pts.push_back(strip_pts2[i]);
    }
    cv::RotatedRect board_rect = cv::minAreaRect(strip_pts);
    solve_pnp(board_rect);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert to OpenCV image (BGR)
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image;
    cv::cvtColor(frame,frame,cv::COLOR_RGB2BGR);
    processFrame(frame);
  } // end callback

  void videoProcess()
  {
    cv::Mat frame;
    while(rclcpp::ok()&&capture_.read(frame))
    {
      processFrame(frame);
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectorAndSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
