#include "Robot.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tag_msgs/action/move_to_pose.hpp"  // Remplace par ton package

using MoveToPose = tag_msgs::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

class JacobianControlActionServer : public rclcpp::Node
{
public:
    explicit JacobianControlActionServer() : Node("jacobian_control_action_server")
    {
         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Crée le serveur d'action
        this->action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "move_to_pose",
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const MoveToPose::Goal> goal) {
                return this->handle_goal(uuid, goal);
            },
            [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
                return this->handle_cancel(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
                return this->handle_accepted(goal_handle);
            });

        // // Abonnement aux joint_states pour mettre à jour l'état du robot
        // subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10,
        //     [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        //         this->callback_joint_states(msg);
        //     });

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_angles", 10);

        // Initialise le robot
        robot_ = std::make_shared<Robot>();

        joint_value_.data.resize(6, 0.0);
        joint_value_.data[0] = 0.0;
        joint_value_.data[1] = 1.57;
        joint_value_.data[2] = 0.0;
        joint_value_.data[3] = 0.0;
        joint_value_.data[4] = 0.0;
        joint_value_.data[5] = 0.0;


        publisher_->publish(joint_value_);
    }

private:
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
    std::shared_ptr<Robot> robot_;
    sensor_msgs::msg::JointState msg_joint_states_;
    std_msgs::msg::Float64MultiArray joint_value_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Nouvelle cible reçue : (%.2f, %.2f, %.2f)",
                    goal->target_pose.pose.position.x,
                    goal->target_pose.pose.position.y,
                    goal->target_pose.pose.position.z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Requête d'annulation reçue");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        // Exécute la tâche dans un thread séparé
        std::thread{std::bind(&JacobianControlActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();

        // Initialise les angles des articulations
        joint_value_.data.resize(6, 0.0);

        // Boucle de contrôle
        for (int i = 0; i < 100 && rclcpp::ok(); ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->message = "Tâche annulée";
                goal_handle->canceled(result);
                return;
            }

            // Récupère la pose cible
            Eigen::Matrix<double, 3, 1> target_pos;
            target_pos(0) = goal_handle->get_goal()->target_pose.pose.position.x;
            target_pos(1) = goal_handle->get_goal()->target_pose.pose.position.y;
            target_pos(2) = goal_handle->get_goal()->target_pose.pose.position.z;

            // Récupère la pose actuelle
            Eigen::Matrix<double, 3, 1> actual_pos;
            // RCLCPP_INFO_STREAM(this->get_logger(),"EFFECTOR FRAME");
            // RCLCPP_INFO_STREAM(this->get_logger(), goal_handle->get_goal()->effector_frame);
/*
            try
            {

                tf_buffer_->canTransform("base_link", "MGD", tf2::TimePointZero, tf2::durationFromSec(2.0));

                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    "base_link",   // frame parent
                    // "MGD",
                    goal_handle->get_goal()->effector_frame,       // frame enfant
                    tf2::TimePointZero  // dernière transform dispo
                );

                actual_pos(0) = t.transform.translation.x;
                actual_pos(1) = t.transform.translation.y;
                actual_pos(2) = t.transform.translation.z;

                // Calcule les nouveaux angles des articulations
                feedback->distance_remaining = robot_->ComputeControl(joint_value_, actual_pos, target_pos, joint_value_);

                publisher_->publish(joint_value_);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(), "Transform non disponible : %s", ex.what());
            }*/


            Transformation T = robot_->ModGeoDirect(joint_value_);
            actual_pos = T.position;

            feedback->distance_remaining = robot_->ComputeControl(joint_value_, actual_pos, target_pos, joint_value_);

            publisher_->publish(joint_value_);


            // Met à jour le feedback
            feedback->current_pose.pose.position.x = actual_pos(0);
            feedback->current_pose.pose.position.y = actual_pos(1);
            feedback->current_pose.pose.position.z = actual_pos(2);
            feedback->current_joint_angles = joint_value_;

            goal_handle->publish_feedback(feedback);



            // Critère d'arrêt si la distance est suffisamment petite
            if (feedback->distance_remaining < 0.005)  // Seuil de 1 cm
            {
                result->success = true;
                result->message = "Cible atteinte";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Cible atteinte !");
                return;
            }

            // Attend 100ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Tâche terminée
        result->success = true;
        result->message = "Tâche terminée avec succès";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Tâche terminée");
    }

    // void callback_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     msg_joint_states_ = *msg;
    //     if (msg_joint_states_.position.size() >= 6)
    //     {
    //         joint_value_.data.resize(6);
    //         for (int i = 0; i < 6; i++)
    //             joint_value_.data[i] = msg_joint_states_.position[i];
    //     }
    // }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JacobianControlActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
/*

#include "Robot.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class JacobianControlNode : public rclcpp::Node
{
public:
    JacobianControlNode() : Node("check_jacobian_control")
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
    auto node = std::make_shared<JacobianControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/
