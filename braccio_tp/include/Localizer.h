#ifndef __LOCALIZER_H__
#define __LOCALIZER_H__

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tag_msgs/msg/tag_pose_array.hpp>
#include <yaml-cpp/yaml.h>
#include "Object.h"

class Localizer
{
public:
    Localizer(rclcpp::Node* node,unsigned int nb_cam = 1);
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
    
    void ReceiveTagInformation( const tag_msgs::msg::TagPoseArray& msg,
                                    unsigned int camera_id = 0);           
    
    
private:
    
    // YAML read the informations about the static marquers
    void AddObjectStaticMarkers(const YAML::Node& node);
    
    void AddReferenceStaticMarkers(const YAML::Node& node);
      
    bool FoundMarker(   const tag_msgs::msg::TagPoseArray& msg,
                        unsigned int id,
                        Transformation & marker);
    
    
    marker ReadMarkerInfo( const YAML::Node& node);
    
    unsigned int nb_cameras;    // number of possible camera used.
    
    bool publish_markers_on_tf = true;
    
    
    // std::vector< Object > objects;  // list des objets mobiles.
    std::vector< Transformation> cameras_poses; // pose of the camera in the world frame.
    std::vector< marker > reference_markers; // define the pose of static markers in world frames
    
    rclcpp::Node* node_;
    // to publish on TF
    std::string ref = "base_link";
    // tf::TransformBroadcaster br;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
     // std::vector<geometry_msgs::msg::TransformStamped> frame_vector_;

};

#endif
