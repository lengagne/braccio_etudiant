#ifndef __LOCALIZER_H__
#define __LOCALIZER_H__

#include "Object.h"


class Localizer
{
public:
    Localizer(ros::NodeHandle* nodehandle,unsigned int nb_cam = 1);
    ~Localizer();
       
    void InitStaticMarkers(const std::string & filename);
    
    void PublishMarkersOnTF( bool b = true)
    {
        publish_markers_on_tf = b;
    }
    
    void PublishTF();
        
    void SetNbCamera( unsigned int nb)
    {
        nb_cameras = nb;
        cameras_poses.resize(nb_cameras);
    }
    
    void ReceiveArucoInformation( const aruco_msgs::MarkerArray& msg,
                                    unsigned int camera_id = 0);           
    
    
private:
    
    // YAML read the informations about the static marquers
    void AddObjectStaticMarkers(const YAML::Node& node);
    
    void AddReferenceStaticMarkers(const YAML::Node& node);
      
    bool FoundMarker(   const aruco_msgs::MarkerArray& msg,
                        unsigned int id,
                        Transformation & marker);
    
    
    marker ReadMarkerInfo( const YAML::Node& node);
    
    unsigned int nb_cameras;    // number of possible camera used.
    
    bool publish_markers_on_tf = true;
    
    
    std::vector< Object > objects;  // list des objets mobiles.
    std::vector< Transformation> cameras_poses; // pose of the camera in the world frame.
    std::vector< marker > reference_markers; // define the pose of static markers in world frames
    
    ros::NodeHandle n;
    // to publish on TF
    std::string ref = "base_link";
    tf::TransformBroadcaster br;
    std::vector<tf::StampedTransform> frame_vector;

};

#endif
