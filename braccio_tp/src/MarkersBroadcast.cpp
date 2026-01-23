#include "Localizer.h"
#include <memory>

class MarkersBroadcastNode : public rclcpp::Node
{
public:
    MarkersBroadcastNode() : Node("markers_broadcast")
    {
        camera_localize_ = std::make_shared<Localizer>(this);
        
        // Declare and get parameter
        this->declare_parameter<std::string>("static_markers", "");
        std::string static_markers_file;
        
        if (this->get_parameter("static_markers", static_markers_file) && !static_markers_file.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Got param: %s", static_markers_file.c_str());
            camera_localize_->InitStaticMarkers(static_markers_file);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get param 'static_markers'");
        }
        
        // Create subscription
        subscription_ = this->create_subscription<tag_msgs::msg::TagPoseArray>(
            "/tag_list",
            10,
            [this](const tag_msgs::msg::TagPoseArray::SharedPtr msg) {
                this->callback(msg);
            });
    }

private:
    void callback(const tag_msgs::msg::TagPoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "callback STEP 1");
        camera_localize_->ReceiveTagInformation(*msg);
        RCLCPP_INFO(this->get_logger(), "callback STEP 2");
        camera_localize_->PublishTF();
        RCLCPP_INFO(this->get_logger(), "callback STEP 3");
    }
    
    std::shared_ptr<Localizer> camera_localize_;
    rclcpp::Subscription<tag_msgs::msg::TagPoseArray>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkersBroadcastNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
