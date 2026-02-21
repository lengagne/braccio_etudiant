#include "Robot.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class CheckMGDNode : public rclcpp::Node
{
public:
    CheckMGDNode() : Node("check_jacobian_control")
    {
        robot_ = std::make_shared<Robot>();

        subscription_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",10,[this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->callback_target(msg);
        });

        subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_state",10,[this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            this->callback_joint_states(msg);
        });

        // Crée un timer qui appelle `timer_callback` toutes les 100 ms
        timer_ = this->create_wall_timer(   std::chrono::milliseconds(100),
            [this]() { this->timer_control_callback(); });  // Appelle la méthode timer_callback

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_angles", 10);


        joint_value_.data.push_back(0.0);
        joint_value_.data.push_back(0.0);
        joint_value_.data.push_back(0.0);
        joint_value_.data.push_back(0.0);
        joint_value_.data.push_back(0.0);
        joint_value_.data.push_back(0.0);

    }

private:

    void timer_control_callback()
    {
        Eigen::Matrix<double,3,1>  target_pos;
        target_pos(0) = msg_target_pose_.pose.position.x;
        target_pos(1) = msg_target_pose_.pose.position.y;
        target_pos(2) = msg_target_pose_.pose.position.z;


        Eigen::Matrix<double,3,1>  actual_pos;
        Transformation T =  robot_->ModGeoDirect(joint_value_);
        actual_pos = T.position;

        robot_->ComputeControl(joint_value_,actual_pos,target_pos,joint_value_);

        publisher_->publish(joint_value_);
    }


    void callback_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        msg_target_pose_ = *msg;
    }

    void callback_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg_joint_states_ = *msg;
        for (int i=0;i<6;i++)
            joint_value_.data.push_back(msg_joint_states_.position[i]);
    }

    std::shared_ptr<Robot> robot_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_target_;
    geometry_msgs::msg::PoseStamped msg_target_pose_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
    sensor_msgs::msg::JointState msg_joint_states_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Float64MultiArray joint_value_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CheckMGDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


