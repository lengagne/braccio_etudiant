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
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"

#include "Robot.h"

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
}
