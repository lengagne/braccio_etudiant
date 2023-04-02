//      test_button.cpp
//      Copyright (C) 2012 lengagne (lengagne@gmail.com)
//
//      This program is free software: you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation, either version 3 of the License, or
//      (at your option) any later version.
//
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//
//      You should have received a copy of the GNU General Public License
//      along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//	from 2020:  Université Clermont Auvergne : Institut Pascal / axis : ISPR / theme MACCS

#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

std_msgs::UInt8MultiArray pose1;
sensor_msgs::JointState p1;

ros::Publisher pub_robot, pub_state;

void callback(const std_msgs::Float64MultiArray& msg)
{
    for (int i=0;i<6;i++)
    {
        if(msg.data[i] > 0)
            pose1.data[i] = msg.data[i] * 180 / 3.14;
        else
            pose1.data[i] = 0;
        p1.position[i] = msg.data[i];        
    }
    p1.position[6] = p1.position[5];  

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_mover");
    
    
    ros::NodeHandle nh;
    pub_robot = nh.advertise<std_msgs::UInt8MultiArray>("/joint_array", 1000);
    pub_state = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    
    ros::Subscriber sub = nh.subscribe("/joint_angles", 1, callback);
    
    pose1.data.resize(6);
    pose1.data[0] = 0;
    pose1.data[1] = 90;
    pose1.data[2] = 0;
    pose1.data[3] = 0;
    pose1.data[4] = 0;
    pose1.data[5] = 0;
    

    p1.name.push_back("base_joint");
    p1.name.push_back("shoulder_joint");
    p1.name.push_back("elbow_joint");
    p1.name.push_back("wrist_pitch_joint");
    p1.name.push_back("wrist_roll_joint");
    p1.name.push_back("gripper_joint");
    p1.name.push_back("sub_gripper_joint");
    
    p1.position.push_back(0.0);
    p1.position.push_back(1.57);
    p1.position.push_back(0);
    p1.position.push_back(0);
    p1.position.push_back(0);
    p1.position.push_back(0.0);
    p1.position.push_back(0.0);    
    

    ros::Rate r(10);
    while (nh.ok())
    {
        p1.header.stamp = ros::Time::now();        
        pub_robot.publish(pose1);
        pub_state.publish(p1);
        ros::spinOnce();
        r.sleep();
    }   
    
    
    ros::spin();
    
    return 0;    
    
}
