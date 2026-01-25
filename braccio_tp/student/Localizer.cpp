#include "Localizer.h"

void Localizer::ReceiveTagInformation(  const tag_msgs::msg::TagPoseArray& msg,
                                        unsigned int camera_id)
{
    // la camera percoit "nb_markers" marqueurs.
    unsigned int nb_markers = msg.tags.size();
    Transformation Trans;
    
    // on localise la caméra par rapport au premier marqueur fixe qui est vu.
    int first_id_seen = -1;
    for (unsigned int i=0;i<reference_markers.size();i++)
    {
        if (FoundMarker(msg,reference_markers[i].id,Trans))   if (reference_markers[i].already_seen)    // on vérifie qu'on a deja vu le marqueur
        {
            first_id_seen = i;
            // A Completer
            // calcul de la pose de la camera  dans le repere monde : cameras_poses[camera_id]
            // en fonction de la pose du marqueur de reference dans le repere monde : reference_markers[i].pose
            // et de la pose du marqueur de reference dans le repere camera : Trans
            // RCLCPP_INFO_STREAM(node_->get_logger(), "marker("<<reference_markers[i].id<<") = " << reference_markers[i].pose);
            cameras_poses[camera_id] = reference_markers[i].pose * Trans.inverse();
            
            break; // pour arreter la boucle for des qu'on trouve un marqueur
        }
    }    
    
    // maintenant on localise tous les autres marqueurs fixes dans le repere 1 (si ils ne sont pas connus)
    for (unsigned int i=first_id_seen+1;i<reference_markers.size();i++)
    {
        if (FoundMarker(msg,reference_markers[i].id,Trans))   if (reference_markers[i].known_reference == false)
        {
            reference_markers[i].already_seen = true;    
            // A completer 
            // calcul de la pose du marqueur de reference dans le repere monde : reference_markers[i].pose
            // en fonction de la pose de la camera  dans le repere monde : cameras_poses[camera_id]
            // et de la pose du marqueur de reference dans le repere camera : Trans
           reference_markers[i].pose = cameras_poses[camera_id]*Trans;
        }
    }      

//     // maintenant on regarde les objets
//     for (int i=0;i<objects.size();i++)
//     {
//         objects[i].ReInitPoseDefined();
//         std::vector<unsigned int> ids = objects[i].GetIds();
//         unsigned int nb = ids.size();
//         for (int j=0;j<nb;j++)
//         {
//             // pour chaque objet on regarde si on a vu un des marqueurs
//             if( FoundMarker(msg,ids[j],Trans))
//             {
//                 Transformation local_pose = objects[i].GetLocalPose(j);
//                 // Calcul de la pose de l'objet dans le repere monde : object_pose
//                 // en fonction de la pose de la camera dans le repere monde : cameras_poses[camera_id]
//                 // de la pose du marqueur dans le repere de l'objet : local_pose
//                 // et de la pose du marqueur dans le repere camera : Trans
//                 Transformation object_pose; // =
//
//                 // on stoque la valeur
//                 objects[i].SetGlobalPose( object_pose);
//             }
//         }
//     }
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////Ne pas modifier sous cette ligne////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

Localizer::Localizer(rclcpp::Node* node,unsigned int nb_cam ): node_(node), nb_cameras(nb_cam)
{
    SetNbCamera(nb_cam);
    // pub_objects = n.advertise<auro8_tps::ObjectPoseArray> ( "/objects", 1);
    // pub_pairs = n.advertise<auro8_tps::ObjectErrorArray> ( "/pairs", 1);

    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node);
}

Localizer::~Localizer()
{

}

void Localizer::AddObjectStaticMarkers(const YAML::Node& node)
{
    // Object new_object( node["name"].as<std::string>() ) ;
    // YAML::Node config = node;
    // for (YAML::const_iterator it=config.begin();it!=config.end();++it)
    // {
    //     if ( it->first.as<std::string>() == "marker" )
    //     {
    //         new_object.add_marker(ReadMarkerInfo(it->second));
    //     }
    // }
    // objects.push_back(new_object);
}

void Localizer::AddReferenceStaticMarkers(const YAML::Node& node)
{
    YAML::Node config = node;
    for (YAML::const_iterator it=config.begin();it!=config.end();++it) 
    {
        reference_markers.push_back( ReadMarkerInfo(it->second)) ;
    }
    
    // on est obligé de connaitre le premier
    reference_markers[0].already_seen = true;
}

bool Localizer::FoundMarker(    const tag_msgs::msg::TagPoseArray& msg,
                                unsigned int id,
                                Transformation & marker)
{
    unsigned int nb_markers = msg.tags.size();
    for (int i=0;i<nb_markers;i++)
    {
        const tag_msgs::msg::TagPose& m = msg.tags[i];
        if (m.id == id)
        {
            marker = Transformation(m.pose.pose);
            return true;
        }
    }
    return false;
}

void Localizer::InitStaticMarkers(const std::string & filename )
{      
    RCLCPP_INFO(node_->get_logger(), "Filename = %s", filename.c_str());
    YAML::Node config = YAML::LoadFile(filename);          
    for (YAML::const_iterator it=config.begin();it!=config.end();++it) 
    {
        if ( it->first.as<std::string>() == "static" )
        {
            RCLCPP_INFO(node_->get_logger(), "AddReferenceStaticMarkers");
            AddReferenceStaticMarkers(it->second);
        // }else if ( it->first.as<std::string>() == "object" )
        // {
        //     RCLCPP_INFO(node_->get_logger(), "AddObjectStaticMarkers");
        //     AddObjectStaticMarkers(it->second);
        // }else if ( it->first.as<std::string>() == "pair" )
        // {
        //     RCLCPP_INFO(node_->get_logger(), "AddPair");
        //     AddPair(it->second);
        }
    }
}

// void Localizer::PublishObject()
// {
//     ROS_INFO("in PublishObject");
//     auro8_tps::ObjectPoseArray list_objects;
//     list_objects.header.frame_id = ref;
//     list_objects.header.stamp = ros::Time::now();
//     list_objects.objects.clear();
//     for (int i=0;i<objects.size();i++)   if( objects[i].IsDefined())
//     {
//         list_objects.objects.push_back(objects[i].GetPose());
//     }
//
//     pub_objects.publish( list_objects);
// }

// void Localizer::PublishPair()
// {
//     auro8_tps::ObjectErrorArray list_pairs;
//     list_pairs.header.frame_id = ref;
//     list_pairs.header.stamp = ros::Time::now();
//     list_pairs.errors.clear();
//     for (int i=0;i<ListOfPairs.size();i++)
//     {
//         int id1 = ListOfPairs[i].id1;
//         int id2 = ListOfPairs[i].id2;
//         if( objects[id1].IsLocalDefined() && objects[id2].IsLocalDefined())
//         {
//             Transformation t1 = objects[id1].GetGlobalPose();
//             Transformation t2 = objects[id2].GetGlobalPose();
//
//             auro8_tps::ObjectError p;
//             p.name = ListOfPairs[i].name;
//             p.position.x = t2.position(0) - t1.position(0);
//             p.position.y = t2.position(1) - t1.position(1);
//             p.position.z = t2.position(2) - t1.position(2);
//             list_pairs.errors.push_back(p);
//         }
//     }
//
//     if (list_pairs.errors.size())
//     {
//         pub_pairs.publish( list_pairs);
//     }
// }

void Localizer::PublishTF()
{
    // frame_vector_.clear();
    
    // publie reperes cameras
    for (int i=0;i<nb_cameras;i++)
    {      
        // frame_vector_.push_back(tf::StampedTransform(cameras_poses[i].convertToTF(), ros::Time::now(),ref,"camera_"+std::to_string(i)));

        auto transform = cameras_poses[i].convertToTransformStamped( ref, "camera_"+std::to_string(i), node_->now());
        br_->sendTransform(transform);
    }
    
    // publie repere marqueur fixes
    for (int i=0;i<reference_markers.size();i++)    if (reference_markers[i].already_seen)
    {
        // frame_vector_.push_back(tf::StampedTransform(reference_markers[i].pose.convertToTF(), ros::Time::now(),ref,"marker_"+std::to_string(reference_markers[i].id)));
        auto transform = reference_markers[i].pose.convertToTransformStamped( ref, "marker_"+std::to_string(reference_markers[i].id), node_->now());

        br_->sendTransform(transform);
    }
    
    // for (int i=0;i<objects.size();i++)  if( objects[i].IsDefined())
    // {
    //     frame_vector_.push_back(tf::StampedTransform(objects[i].GetGlobalPose().convertToTF(), ros::Time::now(),ref,objects[i].GetName()));
    // }
    
    // envoie les informations sur /tf
    // br_.sendTransform(frame_vector_);

}


marker Localizer::ReadMarkerInfo( const YAML::Node& node)
{
    marker out;
    out.id = node["id"].as<int>();
    
    double x = node["position"]["x"].as<double>();
    double y = node["position"]["y"].as<double>();
    double z = node["position"]["z"].as<double>();
    
    double roll = node["orientation"]["roll"].as<double>();
    double pitch = node["orientation"]["pitch"].as<double>();
    double yaw = node["orientation"]["yaw"].as<double>();    
    
    out.pose = Transformation(x,y,z,roll,pitch,yaw);
    
    out.known_reference = node["known_pose"].as<bool>();
    
    out.already_seen = out.known_reference;
    

    RCLCPP_INFO(node_->get_logger(), "Read marker %d ", out.id);

    return out;
}



