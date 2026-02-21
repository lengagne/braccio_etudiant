#include "Robot.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class CheckMGDNode : public rclcpp::Node
{
public:
    CheckMGDNode() : Node("check_mgi")
    {
        robot_ = std::make_shared<Robot>();

        subscription_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",10,[this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->callback_target(msg);
        });

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("angles", 10);

    }

private:


    void callback_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Matrix<double,3,1>  target_pos;
        target_pos(0) = msg->pose.position.x;
        target_pos(1) = msg->pose.position.y;
        target_pos(2) = msg->pose.position.z;

        std_msgs::msg::Float64MultiArray Q;

        Eigen::Matrix<double,3,1>  actual_pos;
        // Transformation T =  robot_->ModGeoDirect(*msg);
        // actual_pos = T.pos;

        // robot_->ComputeControl(Q,actual_pos,target_pos,Q);

        publisher_->publish(Q);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f, %f, %f , %f , %f'", Q.data[0], Q.data[1], Q.data[2],Q.data[4],Q.data[5]);

    }

    std::shared_ptr<Robot> robot_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_target_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CheckMGDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


