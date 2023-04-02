#include <ros/ros.h>
#include <braccio_tp/Target.h>
#include <std_msgs/Float64.h>

uint cpt = 0;

std::vector<braccio_tp::Target> list_msg;
ros::Publisher pub_target;

void errorCallback(const std_msgs::Float64& msg)
{
//     std::cout<<"Receive "<< msg.data<<std::endl;
    if (msg.data<0.02)
    {
        cpt++;
        if (cpt < list_msg.size())
        {
            ROS_INFO("new target");
            pub_target.publish(list_msg[cpt]);
            ros::Duration(0.5).sleep();
        }
    }
}


int main(int argc, char** argv){
   
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle nh;
    
    // init messages
    braccio_tp::Target msg;
    
    // go to the cube 
    msg.object = "/EndEffector";
    msg.target = "/CubeJaune";
    msg.x_offset = 0.0;
    msg.y_offset = 0.0;
    msg.z_offset = 0.0;
    msg.gripper_open = true;  
    
    list_msg.push_back(msg);
    
    msg.target = "/CubeVert";
    msg.x_offset = 0.0;
    msg.y_offset = 0.0;
    msg.z_offset = 0.08;
    msg.gripper_open = false;          
    list_msg.push_back(msg);
    
    msg.gripper_open = true;
    list_msg.push_back(msg);    
    
    
    ros::Subscriber sub_joy = nh.subscribe("/control_error", 10, errorCallback);
    pub_target = nh.advertise<braccio_tp::Target>("control_target", 1000);
    
    ros::spin();  
        
    return 0;
};


