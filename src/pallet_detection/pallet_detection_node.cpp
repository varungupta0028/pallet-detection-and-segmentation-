#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime/core/providers/cuda/cuda_provider_factory.h>
#include <onnxruntime/core/providers/cpu/cpu_provider_factory.h>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#include <fstream>

class ObjectDetectionNode : public rclcpp::Node {
public:
    ObjectDetectionNode() : Node("object_detection_node"), detection_session_(nullptr), segmentation_session_(nullptr) {
        // Declare parameters
        this->declare_parameter<std::string>("weight_folder", "");

        // Subscribe to image topics
        rgb_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&ObjectDetectionNode::rgbCallback, this, std::placeholders::_1)
        );
        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&ObjectDetectionNode::depthCallback, this, std::placeholders::_1)
        );

        // Retrieve the weight folder path
        std::string weight_folder;
        this->get_parameter("weight_folder", weight_folder);

        // Define model paths
        std::string detection_model_path = weight_folder + "/detect_weight/best.onnx";
        std::string segmentation_model_path = weight_folder + "/segment_weight/best.onnx";

        // Load YOLO models
        if (!loadModel(detection_model_path, detection_session_, detection_env_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load detection model: %s", detection_model_path.c_str());
            return;
        }
        if (!loadModel(segmentation_model_path, segmentation_session_, segmentation_env_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load segmentation model: %s", segmentation_model_path.c_str());
            return;
        }
    }

private:
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;

    // Latest depth image
    cv::Mat depth_image_;

    // ONNX Runtime for YOLO
    Ort::Env detection_env_;
    Ort::Env segmentation_env_;
    std::unique_ptr<Ort::Session> detection_session_;
    std::unique_ptr<Ort::Session> segmentation_session_;

    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS2 image message to OpenCV
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Perform detection and segmentation
        cv::Mat detection_results = detect(frame, detection_session_);
        cv::Mat segmentation_results = segment(frame, segmentation_session_);

        // Display results
        cv::imshow("Detected Objects", detection_results);
        cv::imshow("Segmentation", segmentation_results);
        cv::imshow("Original Frame", frame);
        cv::waitKey(1);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS2 depth image to OpenCV
        depth_image_ = cv_bridge::toCvShare(msg, "passthrough")->image;
        RCLCPP_INFO(this->get_logger(), "Depth image received!");
    }

    bool loadModel(const std::string &model_path, std::unique_ptr<Ort::Session> &session, Ort::Env &env) {
        // Check if the file exists
        std::ifstream file(model_path);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "Model file not found: %s", model_path.c_str());
            return false;
        }

        // Load the ONNX model
        Ort::SessionOptions session_options;
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_BASIC);
        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        RCLCPP_INFO(this->get_logger(), "Successfully loaded model: %s", model_path.c_str());
        return true;
    }

    cv::Mat detect(const cv::Mat &frame, const std::unique_ptr<Ort::Session> &session) {
        // Dummy function: Add your ONNX YOLO detection logic here
        cv::Mat annotated_frame = frame.clone();
        cv::putText(annotated_frame, "Detection results", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        return annotated_frame;
    }

    cv::Mat segment(const cv::Mat &frame, const std::unique_ptr<Ort::Session> &session) {
        // Dummy function: Add your ONNX YOLO segmentation logic here
        cv::Mat annotated_frame = frame.clone();
        cv::putText(annotated_frame, "Segmentation results", cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        return annotated_frame;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectionNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
