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
#include "detect_and_solve/model_runtime.hpp"

//defined in recognition.hpp, scale down Mat size to optimize efficiency
extern const float scale;

// #define DEBUG_MODE

class DetectorAndSolver : public rclcpp::Node
{
public:
  DetectorAndSolver():Node("detect_and_solve")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing detect_and_solve node...");

    std::string package_share_dir = ament_index_cpp::get_package_share_directory("detect_and_solve");

    // params: path for config.json, video path or topic path, use_video option

    declare_parameter<std::string>("config_json", package_share_dir +"/config/config.json");
    declare_parameter<std::string>("model_folder", "/home/leoxia/Code/XJTU-RMV-Task05/src/training");
    declare_parameter<std::string>("path", "/camera/image_raw");
    declare_parameter<bool>("use_video", false);

    
    config_json_ = this->get_parameter("config_json").as_string();
    model_folder_ = this->get_parameter("model_folder").as_string();
    use_video=this->get_parameter("use_video").as_bool();
    path_ = this->get_parameter("path").as_string();

    //print!
    RCLCPP_INFO(this->get_logger(),"config.json path: %s", config_json_.c_str());
    RCLCPP_INFO(this->get_logger(),"model folder: %s", model_folder_.c_str());
    if(use_video)
      RCLCPP_INFO(this->get_logger(),"topic: %s", path_.c_str());
    else
      RCLCPP_INFO(this->get_logger(),"video path: %s", path_.c_str());

    if (!loadCameraParams(config_json_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load intrinsics and distortion parameters from: %s", config_json_.c_str());
      throw std::runtime_error("config load failure");
    }
    if (!md_.LoadModel(model_folder_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load model from: %s", model_folder_.c_str());
      throw std::runtime_error("model load failure");
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
    
    // Video or Subscriber
    if (use_video)
    {
      capture_.open(path_);  // from file
      if (!capture_.isOpened())
      {
          RCLCPP_ERROR(get_logger(), "Failed to open video file: %s", path_.c_str());
          rclcpp::shutdown();
          return;
      }
    }
    else
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          path_, 10, std::bind(&DetectorAndSolver::imageCallback, this, std::placeholders::_1));
    }

    //Publishers
    detector_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/image_marked", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detector/pose", 10);
    #ifdef DEBUG_MODE
    np_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/number_pattern", 10);
    #endif

    RCLCPP_INFO(this->get_logger(), "detect_and_solve node started. Board size: %.2f x %.2f mm",
                config_json_.c_str(), board_w_, board_h_);

    // for video input only. as for subscription, spin will handle
    if(use_video)
      videoProcess();
  }

private:
  // Parameters
  std::string config_json_, model_folder_, path_;
  ModelDriver md_;
  double board_w_, board_h_;
  bool use_video;

  // OpenCV camera params
  cv::Mat cameraMatrix_, distCoeffs_;
  cv::Mat newCameraMatrix_; // optional optimized intrinsics

  // OpenCV video capture
  cv::VideoCapture capture_;

  // ROS subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detector_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;// Publisher for pose (optional, useful)
  #ifdef DEBUG_MODE
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr np_pub_;//Publisher for number pattern roi image
  #endif

  bool loadCameraParams(const std::string &json_path)
  {
    //check existence
    if (!std::filesystem::exists(json_path))
    {
      RCLCPP_ERROR(this->get_logger(), "camera json does not exist: %s", json_path.c_str());
      return false;
    }

    //try to open
    cv::FileStorage fs(json_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "failed to open json: %s", json_path.c_str());
      return false;
    }

    // dump camera_matrix
    if (fs["camera_matrix"].isNone())
    {
      RCLCPP_ERROR(this->get_logger(), "camera_matrix not found in json");
      return false;
    }
    fs["camera_matrix"] >> cameraMatrix_;

    cameraMatrix_.at<double>(0, 0) *= scale;
    cameraMatrix_.at<double>(1, 1) *= scale;
    cameraMatrix_.at<double>(0, 2) *= scale;
    cameraMatrix_.at<double>(1, 2) *= scale;

    // distortion is named "distortion_coefficients"
    if (!fs["distortion_coefficients"].isNone())
    {
      fs["distortion_coefficients"] >> distCoeffs_;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "no distortion key found - assume zero");
      distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }

    if (!fs["board_size"].isNone()&&!fs["board_size"]["width_mm"].isNone()&&!!fs["board_size"]["height_mm"].isNone())
    {
      board_w_=(double)fs["board_size"]["width_mm"];
      board_h_=(double)fs["board_size"]["height_mm"];
    }
    else
    {
      // the w x h used here refers to the rect spanned by the two light strips, not the entire black board
      RCLCPP_WARN(this->get_logger(), "no board size specified in json - using default 140.0 x 60.0");
      board_w_=140.0;
      board_h_=60.0;
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

  bool solve_pnp(cv::RotatedRect& rect)
  {
    const double&w = board_w_;
    const double&h = board_h_;
    std::vector<cv::Point2f> rect_corners(4);
    rect.points(rect_corners.data());

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
    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "solvePnP failed");
      return false;
    }

    // tvec is the translation vector: coordinates of the object origin (top-left corner) in camera frame (units mm)
    RCLCPP_INFO(this->get_logger(),
      "PnP success. tvec (mm) = [%.2f, %.2f, %.2f], rvec = [%.4f, %.4f, %.4f]",
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
      rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));

    // Publish PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "camera_link";

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
  }

  void processFrame(cv::Mat originalframe)
  {
    // frame should already be BGR

    // Undistort input frame (optional but recommended)
    cv::Mat undistorted, undistorted_hsv, frame, gray, normImg;

    // Scale down to optimize efficiency
    cv::resize(originalframe, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);

    if (!newCameraMatrix_.empty())
    {
      // if newCameraMatrix_ already computed, use it
      cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_, newCameraMatrix_);
    }
    else
    {
      // compute newCameraMatrix for this image size on first call
      newCameraMatrix_ = cv::getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, frame.size(), 0);
      cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_, newCameraMatrix_);
    }

    // call the 2 rec functions with hsv images
    cv::cvtColor(undistorted, undistorted_hsv, cv::COLOR_BGR2HSV);

    // prepare for the ret value: an array of potential light strips
    // and an array of potential number patterns
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

// display all candid strips and number patterns for debug
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

// pointers for best match
    cv::RotatedRect *pattern,*strip1, *strip2;
    pattern=nullptr;

// local plot Mat variable for debug
    #ifdef DEBUG_MODE
    cv::Mat currplot;
    #endif
    for(auto &ptn: number_patterns)
    {
      #ifdef DEBUG_MODE
      currplot = undistorted.clone();
      plot_rect_red(currplot, ptn);
      cv::imshow("plot", currplot);
      cv::waitKey(0);
      #endif
      // Find out the 2 nearest but not overlapping strips around this pattern
      double dist1,dist2;
      strip1=strip2=nullptr;
      dist1=dist2=100000000.0;
      for(auto &strip: light_strips)
      {
        //if not optimal, pass
        if(cv::norm(ptn.center-strip.center)>=dist2)
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
        cv::Point2f local = strip.center - ptn.center;

        // calc angle and rotation stuff
        float theta = (float)(-ptn.angle * CV_PI / 180.0);  // inverse rotation
        float cosA = std::cos(theta);
        float sinA = std::sin(theta);

        // rotate the point
        float xRot = local.x * cosA - local.y * sinA;
        float yRot = local.x * sinA + local.y * cosA;

        // Check if it's inside the boundary
        float hw = ptn.size.width  * 0.5f;
        float hh = ptn.size.height * 0.5f;
        if(std::abs(xRot) <= hw && std::abs(yRot) <= hh)
        {
          #ifdef DEBUG_MODE
          std::cout<<"pass at 313"<<std::endl;
          #endif
          continue;
        }
        dist2 = cv::norm(ptn.center-strip.center);
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
      std::cout<<ptn.center.x<<" "<<ptn.center.y<<std::endl;
      std::cout<<strip1->center.x<<" "<<strip1->center.y<<std::endl;
      std::cout<<strip2->center.x<<" "<<strip2->center.y<<std::endl;
      #endif

      //Check if this potential number pattern lies in the very middle of the 2 strips
      #define MAX_ERR 50*scale
      if(cv::norm(ptn.center-(strip1->center+strip2->center)*0.5)>MAX_ERR)
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
      pattern=&ptn;
      break;
    }

    if(!pattern||!strip1||!strip2)    //Recognition Failed
      return;

    #ifdef DEBUG_MODE
    std::cout<<"success!"<<std::endl;
    #endif

    //crop to roi for model
    cv::Rect roi_box = pattern->boundingRect();
    roi_box &= cv::Rect(0, 0, undistorted.cols, undistorted.rows); // prevent overflow
    cv::Mat roi = undistorted(roi_box);

    //publish number pattern image for debug
    #ifdef DEBUG_MODE
    std_msgs::msg::Header np_header;//np for number_pattern
    np_header.stamp = this->now();
    np_header.frame_id = "camera_link";
    auto img_msg1 = cv_bridge::CvImage(np_header, "bgr8", resized).toImageMsg();
    np_pub_->publish(*img_msg1);
    #endif DEBUG_MODE

    //run classification model on number pattern image
    std::string cls;
    double prob;
    md_.classify(roi, cls, prob);
    RCLCPP_INFO(this->get_logger(), "Recognized Number: %s; prob: %.4lf",cls.c_str(), prob);

    //plot frame
    cv::RotatedRect strip_ref1,strip_ref2;// ref for refined
    refineStrip(undistorted_hsv, *strip1, strip_ref1);
    refineStrip(undistorted_hsv, *strip2, strip_ref2);
    plot_rect_green(undistorted, strip_ref1);
    plot_rect_green(undistorted, strip_ref2);
    #ifdef DEBUG_MODE
    // debug_frame(undistorted, false);
    #endif
    plot_rect_red(undistorted, *pattern);
    
    #ifdef DEBUG_MODE
    debug_frame(undistorted);
    #endif
    
    cv::Point2f strip_pts1[4], strip_pts2[4];
    std::vector<cv::Point2f> strip_pts;
    strip_ref1.points(strip_pts1);
    strip_ref2.points(strip_pts2);
    for(int i=0;i<4;i++)
    {
      strip_pts.push_back(strip_pts1[i]);
      strip_pts.push_back(strip_pts2[i]);
    }
    cv::RotatedRect board_rect = cv::minAreaRect(strip_pts);
    plot_rect_red(undistorted, board_rect);
    #ifdef DEBUG_MODE
    debug_frame(undistorted, false);
    #endif

    //publish marked image
    std_msgs::msg::Header mk_header;//mk for marked
    mk_header.stamp = this->now();
    mk_header.frame_id = "camera_link";
    auto img_msg2 = cv_bridge::CvImage(mk_header, "bgr8", undistorted).toImageMsg();
    detector_pub_->publish(*img_msg2);

    //solve pnp
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
