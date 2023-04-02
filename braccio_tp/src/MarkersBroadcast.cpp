#include "Localizer.h"

// déclaration des variables globales 
Localizer *camera_localize;


// fonction appeler des qu'on recoit un message de aruco
void callback(const aruco_msgs::MarkerArray& msg)
{
    camera_localize->ReceiveArucoInformation( msg);
    camera_localize->PublishTF();
}


int main(int argc, char** argv){

    
    ros::init(argc, argv, "markers_broadcast");

    ros::NodeHandle nh;
    
    camera_localize = new Localizer(&nh);
    
    std::string static_markers_file;
    
    if (nh.getParam("/braccio_tp/static_markers", static_markers_file))
    {
        ROS_INFO("Got param: %s", static_markers_file.c_str());
        camera_localize->InitStaticMarkers(static_markers_file);
    }
    else
        ROS_ERROR("Failed to get param '/braccio_tp/static_markers'");


    ros::Subscriber sub = nh.subscribe("/aruco_marker_publisher/markers", 10, callback);    
    
    
    ros::spin();
    return 0;
};


