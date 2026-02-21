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
        Eigen::Matrix<double,3,1>  pos;
        pos(0) = msg->pose.position.x;
        pos(1) = msg->pose.position.y;
        pos(2) = msg->pose.position.z;

        std_msgs::msg::Float64MultiArray Q;

        robot_->ModGeoInverse(pos,Q);

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
