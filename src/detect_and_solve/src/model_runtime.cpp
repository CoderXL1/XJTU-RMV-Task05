#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"


class ModelDriver
{
public:
    ModelDriver()
    {
        // 加载 ONNX 模型
        net_ = cv::dnn::readNetFromONNX("/home/neo/Code/XJTU-RMV-Task05/src/training/tiny_lenet.onnx");
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // 加载类别列表
        std::ifstream f("/home/neo/Code/XJTU-RMV-Task05/src/training/classes.json");
        
        for (auto &cls : j) {
            classes_.push_back(cls.get<std::string>());
        }
        negative_class_index_ = classes_.size() - 1;

        // 订阅 ROI 图像 topic
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/detector/number_pattern", 10,
            std::bind(&SymbolDetectorNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "SymbolDetectorNode initialized.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // 转 cv::Mat
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // --- 预处理 ---
        cv::Mat input;
        cv::cvtColor(frame, input, cv::COLOR_BGR2GRAY); // 灰度
        cv::resize(input, input, cv::Size(48,32));       // resize (W,H)
        input.convertTo(input, CV_32F, 1.0/255);        // [0,1]
        input = (input - 0.5) / 0.5;                     // Normalize (-1,1)

        // 转 blob [1,1,H,W]
        cv::Mat blob = cv::dnn::blobFromImage(input);

        // 前向推理
        net_.setInput(blob);
        cv::Mat out = net_.forward(); // shape [1, num_classes]

        // 找最大概率
        cv::Point classIdPoint;
        double maxVal;
        cv::minMaxLoc(out, 0, &maxVal, 0, &classIdPoint);
        int top_class = classIdPoint.x;

        // 阈值判断
        double THRESH = 0.8;
        std::string result;
        if (top_class == negative_class_index_ || maxVal < THRESH) {
            result = "No match";
        } else {
            result = classes_[top_class];
        }

        RCLCPP_INFO(this->get_logger(), "Detected: %s (prob %.3f)", result.c_str(), maxVal);
    }

    cv::dnn::Net net_;
    std::vector<std::string> classes_;
    int negative_class_index_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SymbolDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
