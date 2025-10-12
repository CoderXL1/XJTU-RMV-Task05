#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "detect_and_solve/recognition.hpp"

namespace fs=std::filesystem;

class DetectorAndSolver : public rclcpp::Node
{
public:
  DetectorAndSolver():Node("detect_and_solve")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing detect_and_solve node...");
    // params: path for yaml, size of light strip(mm), use_video option, video path or topic path
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("detect_and_solve");

    this->declare_parameter<std::string>("camera_yaml", package_share_dir +"/config/camera.yaml");
    this->declare_parameter<double>("width_mm", 8.0);
    this->declare_parameter<double>("height_mm", 55.0); 
    declare_parameter<bool>("use_video", false);
    declare_parameter<std::string>("path", "/camera/image_raw");
    
    camera_yaml_ = this->get_parameter("camera_yaml").as_string();
    std::cout<<camera_yaml_<<std::endl;//print!
    rect_w_ = this->get_parameter("width_mm").as_double();
    rect_h_ = this->get_parameter("height_mm").as_double();
    use_video=this->get_parameter("use_video").as_bool();
    path_ = this->get_parameter("path").as_string();
    // if (!loadCameraParams(camera_yaml_)) {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to load intrinsics and distortion parameters from: %s", camera_yaml_.c_str());
    //   throw std::runtime_error("config load failure");
    // }
    //TODO: yaml read
    cameraMatrix_ = (cv::Mat_<double>(3,3) << 
      2584.809123, 0.0, 1540.358948,
      0.0, 2570.871247, 1087.701687,
      0.0, 0.0, 1.0
    );

    distCoeffs_ = (cv::Mat_<double>(1,4) << 
        -0.074792, 0.105821, 0.0, 0.0
    );
    // Subscriber
    if (use_video) {
      capture_.open(path_);  // 从文件取流
      if (!capture_.isOpened()) {
          RCLCPP_ERROR(get_logger(), "Failed to open video file: %s", path_.c_str());
          rclcpp::shutdown();
          return;
      }
    } else {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          path_, 10, std::bind(&DetectorAndSolver::imageCallback, this, std::placeholders::_1));
    }

    // Publisher for pose (optional, useful)
    // pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_pose", 10);

    RCLCPP_INFO(this->get_logger(), "detect_and_solve node started. camera_yaml: %s, rect: %.2f x %.2f mm",
                camera_yaml_.c_str(), rect_w_, rect_h_);

    if(use_video)
      videoProcess();
  }
  private:
  // Parameters
  std::string camera_yaml_;
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
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  bool loadCameraParams(const std::string &yaml_path) {
    if (!fs::exists(yaml_path)) {
      RCLCPP_ERROR(this->get_logger(), "camera yaml does not exist: %s", yaml_path.c_str());
      return false;
    }

    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "failed to open yaml: %s", yaml_path.c_str());
      return false;
    }

    // camera_matrix is named "camera_matrix"
    if (fs["camera_matrix"].isNone()) {
      RCLCPP_ERROR(this->get_logger(), "camera_matrix not found in yaml");
      return false;
    }
    fs["camera_matrix"] >> cameraMatrix_;

    // distortion is named "distortion_coefficients"
    if (!fs["distortion_coefficients"].isNone()) {
      fs["distortion_coefficients"] >> distCoeffs_;
    } else {
      RCLCPP_WARN(this->get_logger(), "no distortion key found - assume zero");
      distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);
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

bool solve_pnp(cv::RotatedRect& rect, cv::Mat undistorted)
  {
    const double&w = rect_w_;
    const double&h = rect_h_;

    cv::Point2f rect_corners_arr[4];
    rect.points(rect_corners_arr);
    std::vector<cv::Point2f> rect_corners(rect_corners_arr, rect_corners_arr + 4);

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

    // Convert rvec to rotation matrix for nicer logging
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // tvec is the translation vector: coordinates of the object origin (top-left corner) in camera frame (units mm)
    RCLCPP_INFO(this->get_logger(),
      "PnP success. tvec (mm) = [%.2f, %.2f, %.2f], rvec = [%.4f, %.4f, %.4f]",
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
      rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));

    // Log rotation matrix
    RCLCPP_DEBUG(this->get_logger(), "Rotation matrix:\n[%f %f %f; %f %f %f; %f %f %f]",
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));

    /*
    // Publish PoseStamped (convert mm -> meters)
    // geometry_msgs::msg::PoseStamped pose_msg;
    // pose_msg.header.stamp = this->now();
    // pose_msg.header.frame_id = "camera_frame"; // set appropriately for your TF tree

    // translation: convert to meters
    pose_msg.pose.position.x = tvec.at<double>(0) / 1000.0;
    pose_msg.pose.position.y = tvec.at<double>(1) / 1000.0;
    pose_msg.pose.position.z = tvec.at<double>(2) / 1000.0;

    // convert rotation matrix to quaternion
    // build tf-like quaternion from R (row-major)
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    double qw, qx, qy, qz;
    if (trace > 0.0) {
      double s = sqrt(trace + 1.0) * 2.0;
      qw = 0.25 * s;
      qx = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
      qy = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
      qz = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
    } else {
      if ((R.at<double>(0,0) > R.at<double>(1,1)) && (R.at<double>(0,0) > R.at<double>(2,2))) {
        double s = sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2)) * 2.0;
        qw = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
        qx = 0.25 * s;
        qy = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
        qz = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
      } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
        double s = sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2)) * 2.0;
        qw = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
        qx = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
        qy = 0.25 * s;
        qz = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
      } else {
        double s = sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1)) * 2.0;
        qw = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
        qx = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
        qy = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
        qz = 0.25 * s;
      }
    }

    pose_msg.pose.orientation.w = qw;
    pose_msg.pose.orientation.x = qx;
    pose_msg.pose.orientation.y = qy;
    pose_msg.pose.orientation.z = qz;

    pose_pub_->publish(pose_msg);*/

    // Optionally draw result and show (if running with GUI)
    // draw corners and axis for visualization
    std::vector<cv::Point2f> axis2d;
    // Project 3D axis points to image for visualization (optional)
    std::vector<cv::Point3f> axis3d = {
      cv::Point3f(0,0,0),
      cv::Point3f((float)rect_w_, 0, 0),
      cv::Point3f(0, (float)rect_h_, 0),
      cv::Point3f(0,0,-(float)rect_w_) // some depth vector
    };
    cv::projectPoints(axis3d, rvec, tvec, newCameraMatrix_, cv::Mat(), axis2d);

    for (size_t i=0;i<axis2d.size();++i) {
      cv::circle(undistorted, axis2d[i], 3, cv::Scalar(0,255,0), -1);
    }
    for (auto &p: rect_corners) {
      cv::circle(undistorted, p, 4, cv::Scalar(0,0,255), -1);
    }
    // show in window (only if you have DISPLAY)
    // debug_frame(undistorted);
  }

  void processFrame(cv::Mat frame)
  {
    // frame should already be BGR
    // Undistort input frame (optional but recommended)
    cv::Mat undistorted;
    if (!newCameraMatrix_.empty()) {
      // if newCameraMatrix_ already computed, use it
      cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_, newCameraMatrix_);
    } else {
      // compute newCameraMatrix for this image size on first call
      newCameraMatrix_ = cv::getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, frame.size(), 0);
      cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_, newCameraMatrix_);
    }

    // Call your recognition routine to extract rectangle corners
    cv::RotatedRect rect1, rect2;
    bool found = recognition_main(undistorted, rect1, rect2); // <-- replace call if your signature differs

    if (!found) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "recognition_main did not find target in this frame");
      return;
    }

    // IMPORTANT: The order of object points must correspond to the order of image points returned.
    // Here we assume recognition_main returns points in order:
    //   0: top-left, 1: top-right, 2: bottom-right, 3: bottom-left
    // If your recognition returns different order, reorder either img_corners or objectPoints accordingly.

    // Construct object points in real-world coordinates (units: mm here)
    solve_pnp(rect1, undistorted.clone());
    solve_pnp(rect2, undistorted.clone());
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
