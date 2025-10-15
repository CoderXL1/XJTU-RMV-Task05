#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "hik_camera/hik_camera_driver.hpp"  // 你自己的驱动头文件

class HikCameraNode : public rclcpp::Node
{
public:
  HikCameraNode()
  : Node("hik_camera_node"), driver_(std::make_shared<HikCameraDriver>())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing HikRobot camera node...\nMAKE SURE TO ADJUST THE APERTURE!");

    // 打开相机
    if (!driver_->openCamera()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
      rclcpp::shutdown();
      return;
    }

    // 设置相机参数
    this->declare_parameter<std::string>("pixel_format", "RGB8Packed");
    this->declare_parameter<double>("gain", 16.0);
    this->declare_parameter<double>("exposure_time", 5000.0);
    this->declare_parameter<double>("fps", 15.0);

    // 获取初始参数
    auto pixel_format = get_parameter("pixel_format").as_string();
    auto gain = get_parameter("gain").as_double();
    auto exposure = get_parameter("exposure_time").as_double();
    auto fps = get_parameter("fps").as_double();

   if(!(driver_->setPixelFormat(pixel_format)&&
    driver_->setGain(gain)&&
    driver_->setExposure(exposure)&&
    driver_->setFPS(fps)
    ))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set camera parameters.");
      rclcpp::shutdown();
      return;
    }

    if(!driver_->appendCallback(HikCameraNode::ImageCallback, this))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to register image callback.");
      rclcpp::shutdown();
      return;
    }
    
    if(!driver_->startGrabbing())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start grabbing frames from camera.");
      rclcpp::shutdown();
      return;
    }

    // 发布者
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    fps_pub_ = this->create_publisher<std_msgs::msg::Float32>("camera/fps", 10);

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::onParamChange, this, std::placeholders::_1)
    );

    // 创建一个定时器，周期性(1s)抓取并发布帧率
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),  // 1000ms / 30fps
      std::bind(&HikCameraNode::timerCallback, this)
    );
    RCLCPP_INFO(this->get_logger(), "Camera node initialized successfully.");
  }

  ~HikCameraNode()
  {
    RCLCPP_INFO(this->get_logger(), "Camera will be closed.");
  }

  rcl_interfaces::msg::SetParametersResult 
  onParamChange(const std::vector<rclcpp::Parameter> &params)
  {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      reconnectCamera();
      return result;
  }
  static void __stdcall ImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
  {
    auto self = static_cast<HikCameraNode*>(pUser);
    MV_FRAME_OUT_INFO_EX &info = *pFrameInfo;

    cv::Mat image;
    std::string encoding;
    if (info.enPixelType == PixelType_Gvsp_Mono8) {
        image = cv::Mat(info.nHeight, info.nWidth, CV_8UC1);
        memcpy(image.data, pData, info.nWidth * info.nHeight);
        encoding = "mono8";
    }
    else if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
        image = cv::Mat(info.nHeight, info.nWidth, CV_8UC3);
        memcpy(image.data, pData, info.nWidth * info.nHeight * 3);
        encoding = "bgr8";
    }
    else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
        // RCLCPP_INFO(self->get_logger(), "RGB8 detected");
        cv::Mat temp(info.nHeight, info.nWidth, CV_8UC3);
        // image = cv::Mat(info.nHeight, info.nWidth, CV_8UC3);
        memcpy(temp.data, pData, info.nWidth * info.nHeight * 3);
        cv::cvtColor(temp, image, cv::COLOR_RGB2BGR);
        encoding = "bgr8";
    }
    else if (info.enPixelType == PixelType_Gvsp_BayerRG8) {
        RCLCPP_INFO(self->get_logger(), "BayerRG8 detected");
        cv::Mat bayer(info.nHeight, info.nWidth, CV_8UC1);
        memcpy(bayer.data, pData, info.nWidth * info.nHeight);
        cv::cvtColor(bayer, image, cv::COLOR_BayerRG2BGR);
        encoding = "bgr8";
    }
    else {
        RCLCPP_ERROR(self->get_logger(), "Unsupported pixel type: %ld", info.enPixelType);
        return;
    }

    // 使用 cv_bridge 转换成 ROS2 消息
    std_msgs::msg::Header header;
    header.stamp = self->now();
    header.frame_id = "camera_link";

    auto msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    //Key Debugging information
    // RCLCPP_INFO(self->get_logger(),
    // "Publishing image: %dx%d, step=%d, data=%zu",
    // msg->width, msg->height, msg->step, msg->data.size());
    self->img_pub_->publish(*msg);
  }

private:

  bool reconnectCamera()
  {
    driver_->closeCamera();

    // 等待一点时间避免接口未释放
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!driver_->openCamera())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reopen camera device.");
      return false;
    }
    // 恢复上次参数
    if (!driver_->setPixelFormat(get_parameter("pixel_format").as_string()))
    {
      RCLCPP_WARN(this->get_logger(), "Failed to restore pixel format setting.");
      return false;
    }
    if (!driver_->setGain(get_parameter("gain").as_double())) 
    {
      RCLCPP_WARN(this->get_logger(), "Failed to restore gain setting.");
      return false;
    }
    if (!driver_->setFPS(get_parameter("fps").as_double()))
    {
      RCLCPP_WARN(this->get_logger(), "Failed to restore frame rate setting.");
      return false;
    }

    if (!driver_->appendCallback(HikCameraNode::ImageCallback, this))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to re-register image callback.");
      return false;
    }
    if (!driver_->startGrabbing())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to restart grabbing.");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Camera reconnected and grabbing restarted.");
    return true;
  }

  void timerCallback()
  {
    const int max_failures = 2;
    static int failure_cnt = 0;
    float fps;
    if(!driver_->getFPS(fps))
    {
      if(failure_cnt<max_failures)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to get FPS from camera driver.");
        failure_cnt++;
      }
      else
      {
        if(failure_cnt==max_failures)
        {
          RCLCPP_ERROR(this->get_logger(), "Camera appears disconnected. Attempting reconnection...");
          failure_cnt++;
        }
        if(!reconnectCamera())
        {
          RCLCPP_FATAL(this->get_logger(), "Reconnection failed. Will retry next cycle.");
        }
        else
          failure_cnt = 0;  // 重置计数
      }
      return;
    }

    auto msg = std_msgs::msg::Float32();
    msg.data = fps;
    fps_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published FPS: %f", fps);
  }

  std::shared_ptr<HikCameraDriver> driver_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fps_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HikCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
