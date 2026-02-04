#include "Robot.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>


class CheckMGDNode : public rclcpp::Node
{
public:
    CheckMGDNode() : Node("check_mgd")
    {
        robot_ = std::make_shared<Robot>();

        // Create subscription
        subscription1_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/joint_angles",10, [this](std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            this->callback_angle(msg);
        }
        );
        subscription2_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            this->callback_state(msg);
        });

        br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void callback_angle(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        Transformation T =  robot_->ModGeoDirect(*msg);

        auto transform = T.convertToTransformStamped( "base_link", "MGD", this->now());
        br_->sendTransform(transform);
    }

    void callback_state(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std_msgs::msg::Float64MultiArray MSG;
        for (int i=0;i<6;i++)
            MSG.data.push_back(msg->position[i]);

        auto msg_ptr = std::make_shared<std_msgs::msg::Float64MultiArray>(MSG);
        callback_angle(msg_ptr);
    }

    std::shared_ptr<Robot> robot_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription2_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CheckMGDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


/*
// The Goal of this node is to compare the observation (via aruco) of the end effector and its computation through the model.

Robot Braccio;

void callback(const std_msgs::Float64MultiArray& msg)
{
    Transformation T =  Braccio.ModGeoDirect(msg);        
    
    // Send the pose to TF
    static tf::TransformBroadcaster br;
    std::vector<tf::StampedTransform> frame_vector;
    frame_vector.push_back(tf::StampedTransform(T.convertToTF(), ros::Time::now(),"base_link","MGD"));
    br.sendTransform(frame_vector);
}

void callback_states(const sensor_msgs::JointState& msg)
{
    std_msgs::Float64MultiArray MSG;
    for (int i=0;i<6;i++)
        MSG.data.push_back(msg.position[i]);
    callback(MSG);     
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "check_MGD");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joint_angles", 10, callback);
    ros::Subscriber sub_js = nh.subscribe("/joint_states", 10, callback_states);
           
    ros::spin();
    
    return 0;        
}*/
