#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <braccio_tp/Target.h>
#include <tf/transform_listener.h>
#include "Robot.h"


Eigen::Matrix<double,3,1> computed_position, current_position, desired_position;

VECTOR q;

Robot Braccio;

std_msgs::Float64 error;

sensor_msgs::Joy joy_message;
bool joy_received = false;

braccio_tp::Target target_message;
bool target_received = false;

void joyCallback(const sensor_msgs::Joy& joy)
{
    joy_message = joy;
    joy_received = true;
}

void targetCallback(const braccio_tp::Target& msg)
{
    target_message = msg;
    target_received = true;
}


int main(int argc, char** argv){
   
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;
    
    // init messages
    q.data.resize(6);
    q.data[0] = 0;
    q.data[1] = 1.57;
    q.data[2] = 0;
    q.data[3] = 0;
    q.data[4] = 0;
    q.data[5] = 0.72;
    
    error.data = 1e3;
    
    target_message.object = "/EndEffector";
    target_message.target = "/CubeJaune";
    target_message.x_offset = 0.0;
    target_message.y_offset = 0.0;
    target_message.z_offset = 0.0;
    target_message.gripper_open = true;    
    
    ros::Subscriber sub_joy = nh.subscribe("/joy", 10, joyCallback);
    ros::Subscriber sub_target = nh.subscribe("/control_target", 10, targetCallback);        
    
    ros::Publisher pub_q = nh.advertise<std_msgs::Float64MultiArray>("joint_angles", 1);
    ros::Publisher pub_error = nh.advertise<std_msgs::Float64>("control_error", 1);
    
    ROS_INFO("start");
    ROS_INFO("Do not forget to push on start button");
    ros::Rate loop_rate(2);
    
    tf::TransformListener listener;
    tf::StampedTransform transform;    
    while (ros::ok())   
    {
        ROS_INFO("Computing");
        try{
            listener.lookupTransform("/base_link", target_message.target, ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
//             ros::Duration(1.0).sleep();
        }        
        desired_position(0) = transform.getOrigin().x() + target_message.x_offset;;
        desired_position(1) = transform.getOrigin().y() + target_message.y_offset;;
        desired_position(2) = transform.getOrigin().z() + target_message.z_offset;

//             std::cout<<"desired_position = "<< desired_position.transpose()<<std::endl;
        
        bool find_actual_position = true;
        
        computed_position = (Braccio.ModGeoDirect(q)).position;
        
        try{
            listener.lookupTransform("/base_link",target_message.object, ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
//             ros::Duration(1.0).sleep();
            find_actual_position = false;
        }        
        current_position(0) = transform.getOrigin().x()+ target_message.x_offset;
        current_position(1) = transform.getOrigin().y()+ target_message.y_offset;;
        current_position(2) = transform.getOrigin().z()+ target_message.z_offset;;
        error.data = 62.0;
        if(find_actual_position)
        {
            if ((current_position - computed_position).norm()<0.05)
            {
                Braccio.ComputeControl(q,current_position,desired_position,q);
                
                error.data = (desired_position-current_position).norm();
            }else
            {
                Braccio.ComputeControl(q,computed_position,desired_position,q);
                error.data = 41.0;
                
                
            }
        }else
        {
            Braccio.ComputeControl(q,computed_position,desired_position,q);
            error.data = 42.0;
        }
//             std::cout<<"current_position = "<< current_position.transpose()<<std::endl;
        
        
        
        if (target_message.gripper_open)
            q.data[5] = 0.0;
        else
            q.data[5] = 1.0;
        
        if(joy_received && joy_message.buttons[9])
            pub_q.publish(q);
        else
            ROS_INFO("No publish");

        pub_error.publish(error);

        
        ros::spinOnce();  
        loop_rate.sleep();        
    }
        
    return 0;
};


