#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode() : Node("image_publisher_node"), index_(0) {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("image_folder", "");
        this->declare_parameter<int>("publish_rate", 1);

        this->get_parameter("image_folder", image_folder_);
        this->get_parameter("publish_rate", publish_rate_);

        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);

        // Timer to publish images at the specified rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate_),
            std::bind(&ImagePublisherNode::publishImage, this)
        );

        // Load image files from the specified folder
        loadImages();

        if (image_files_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No images found in the folder: %s", image_folder_.c_str());
        }
    }

private:
    std::string image_folder_;
    int publish_rate_;
    std::vector<std::string> image_files_;
    size_t index_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void loadImages() {
        if (!fs::exists(image_folder_) || !fs::is_directory(image_folder_)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid folder path: %s", image_folder_.c_str());
            return;
        }

        for (const auto &entry : fs::directory_iterator(image_folder_)) {
            if (entry.is_regular_file()) {
                auto path = entry.path();
                if (path.extension() == ".png" || path.extension() == ".jpg") {
                    image_files_.push_back(path.string());
                }
            }
        }
    }

    void publishImage() {
        if (image_files_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No images available for publishing.");
            return;
        }

        // Load the current image
        const std::string &image_path = image_files_[index_];
        cv::Mat cv_image = cv::imread(image_path);

        if (cv_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
            return;
        }

        // Convert OpenCV image to ROS message
        sensor_msgs::msg::Image::SharedPtr ros_image = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", cv_image
        ).toImageMsg();

        // Publish the image
        publisher_->publish(*ros_image);

        RCLCPP_INFO(this->get_logger(), "Publishing image: %s", image_path.c_str());

        // Move to the next image (loop back to the start if at the end)
        index_ = (index_ + 1) % image_files_.size();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisherNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
