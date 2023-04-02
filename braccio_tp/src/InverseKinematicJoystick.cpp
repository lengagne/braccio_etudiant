#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include "Robot.h"


Eigen::Matrix<double,3,1> position, desired_position;

VECTOR q;

Robot Braccio;

sensor_msgs::Joy joy_message;
bool joy_received = false;


void joyCallback(const sensor_msgs::Joy& joy)
{
    joy_message = joy;
    joy_received = true;
}

// void jointCallback(const sensor_msgs::JointState& joint)
// {
//     for (int i=0;i<6;i++)
//     {
//         if (joint.name[i] == "base_joint")    q.data[0] = joint.position[i];
//         if (joint.name[i] == "shoulder_joint")    q.data[1] = joint.position[i];
//         if (joint.name[i] == "elbow_joint")    q.data[2] = joint.position[i];
//         if (joint.name[i] == "wrist_pitch_joint")    q.data[3] = joint.position[i];
//         if (joint.name[i] == "wrist_roll_joint")    q.data[4] = joint.position[i];
//         if (joint.name[i] == "gripper_joint")    q.data[5] = joint.position[i];
//     }        
// }



int main(int argc, char** argv){
   
    ros::init(argc, argv, "inverse_kinematic");
    ros::NodeHandle nh;
    
    // init messages
    q.data.resize(6);
    q.data[0] = 0;
    q.data[1] = 1.57;
    q.data[2] = 0;
    q.data[3] = 0;
    q.data[4] = 0;
    q.data[5] = 0.72;
    
    joy_message.buttons.resize(12);
    
    desired_position = (Braccio.ModGeoDirect(q)).position;    
    
    ros::Subscriber sub_joy = nh.subscribe("/joy", 1, joyCallback);    
    
    ros::Publisher pub_q = nh.advertise<std_msgs::Float64MultiArray>("joint_angles", 1000);
    
    ros::Publisher pub_m = nh.advertise<visualization_msgs::Marker>("marker_target", 1000);
    
    visualization_msgs::Marker Target;
    Target.header.frame_id = "base_link";
    Target.header.stamp = ros::Time();
    Target.ns = "my_namespace";
    Target.id = 0;
    Target.type = visualization_msgs::Marker::SPHERE;
//     Target.action = visualization_msgs::Marker::ADD;
    Target.pose.position.x = 1;
    Target.pose.position.y = 1;
    Target.pose.position.z = 1;
    Target.pose.orientation.x = 0.0;
    Target.pose.orientation.y = 0.0;
    Target.pose.orientation.z = 0.0;
    Target.pose.orientation.w = 1.0;
    Target.scale.x = 0.02;
    Target.scale.y = 0.02;
    Target.scale.z = 0.02;
    Target.color.a = 1.0; // Don't forget to set the alpha!
    Target.color.r = 0.0;
    Target.color.g = 1.0;
    Target.color.b = 0.0;    
    
    ROS_INFO("start");
    ros::Rate loop_rate(2);
    while( ! joy_received)
    {
        ros::spinOnce();  
        loop_rate.sleep();          
    }
    
    while (ros::ok())
    {
        // update desired position regarding joystick
        desired_position(0) += 0.01* joy_message.axes[1];
        desired_position(1) += 0.01* joy_message.axes[0];        
        desired_position(2) += 0.01*joy_message.buttons[5] - 0.01*joy_message.buttons[7];
        
        Braccio.ModGeoInverse(desired_position,q);
        
        // open/close gripper
        q.data[5] += 0.05*joy_message.buttons[1] - 0.05*joy_message.buttons[3];
        
        pub_q.publish(q);
        ROS_INFO("q(5) = %f", q.data[5]);

        Target.pose.position.x = desired_position(0);
        Target.pose.position.y = desired_position(1);
        Target.pose.position.z = desired_position(2);
        pub_m.publish(Target);
        
        ros::spinOnce();  
        loop_rate.sleep();        
    }
        
    return 0;
};


