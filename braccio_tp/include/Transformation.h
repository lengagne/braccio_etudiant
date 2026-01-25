#ifndef __TRANSFORMATION_H__
#define __TRANSFORMATION_H__

// #include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>

#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>

// retourne une matrice de rotation a partir de roll pitch yaw (en radian)
// a completer dans le fichier Transformation.cpp       
Eigen::Matrix<double,3,3> setRPY( const double & roll, const double & pitch, const double & yaw);


Eigen::Matrix<double,3,3> setQuaternion( const double & a, const double & b, const double & c, const double &d);

// retourne un vecteur (roll, pitch, yaw) a partir d'une matrice de rotation
// a completer dans le fichier Transformation.cpp       
Eigen::Matrix<double,3,1> getRPY(const Eigen::Matrix<double,3,3> mat);


class Transformation
{
public :
        // ne pas changer
        // creation d'une transformation a partir de x,y,z, roll pitch et yaw (exprimé en degré)
        Transformation(const double & x, const double & y, const double & z,  
                       const double & roll=0, const double & pitch=0, const double & yaw=0)
        {
            position(0) = x;
            position(1) = y;
            position(2) = z;
            double pi = 2*asin(1);
            rotation = setRPY(roll*pi/180,pitch*pi/180,yaw*pi/180);
        }
        
        Transformation(const double & x, const double & y, const double & z,  
                       const double & a, const double & b, const double & c, const double & d)
        {
            position(0) = x;
            position(1) = y;
            position(2) = z;
            rotation = setQuaternion(a,b,c,d);
            
        }        
           
        // ne pas changer
        // creation d'une transformation par defaut (identite)    
        Transformation( )
        {
            position = Eigen::Matrix<double,3,1>::Zero();
            rotation = Eigen::Matrix<double,3,3>::Identity();
        }

    
        // ne pas changer
        // copie d'une transformation
        Transformation( const Eigen::Matrix<double,3,1>& pos,
                        const Eigen::Matrix<double,3,3>& rot):position(pos),rotation(rot)
        {
            
        }
        
        // ne pas changer
        // creation d'une transformation a partir de x,y,z, roll pitch et yaw (sous forme de vecteur)
        Transformation( const Eigen::Matrix<double,3,1>& pos,
                        const Eigen::Matrix<double,3,1>& rot):position(pos)
        {
            rotation = setRPY(rot(0),rot(1),rot(2));
        }        
        
        // ne pas changer
        // conversion d'une pose (aruco) en Transformation
        Transformation( const geometry_msgs::msg::Pose& msg)
        {
            double a = msg.orientation.w;
            double b = msg.orientation.x;
            double c = msg.orientation.y;
            double d = msg.orientation.z;
            
            rotation = setQuaternion(a,b,c,d);
            
            position(0) = msg.position.x;
            position(1) = msg.position.y;
            position(2) = msg.position.z;
        }
        
        geometry_msgs::msg::Pose converToPose() const
        {
            geometry_msgs::msg::Pose out;
            out.position.x = position(0);
            out.position.y = position(1);
            out.position.z = position(2);
            
            double a = 0.5*sqrt(rotation(0,0)+rotation(1,1)+rotation(2,2)+1);
            out.orientation.w = a;
            out.orientation.x = (rotation(2,1) - rotation(1,2))/(4*a);
            out.orientation.y = (rotation(0,2) - rotation(2,0))/(4*a);
            out.orientation.z = (rotation(1,0) - rotation(0,1))/(4*a);
            
            
            return out;
        }

        // // ne pas changer
        // // conversion d'une Transformation en tf::Transform
        // tf::Transform convertToTF()
        // {
        //     tf::Matrix3x3 Mq;
        //     for (int i=0;i<3;i++)   for (int j=0;j<3;j++)
        //         Mq[i][j] = rotation(i,j);
        //     return tf::Transform(  Mq, tf::Vector3(position(0),position(1),position(2)));
        // }

    geometry_msgs::msg::TransformStamped convertToTransformStamped(
        const std::string& frame_id,
        const std::string& child_frame_id,
        const builtin_interfaces::msg::Time& timestamp) const
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = timestamp;
        transform.header.frame_id = frame_id;
        transform.child_frame_id = child_frame_id;

        transform.transform.translation.x = position(0);
        transform.transform.translation.y = position(1);
        transform.transform.translation.z = position(2);


        // std::cout<<"rotation = "<< rotation <<std::endl;
        // double a = 0.5*sqrt(rotation(0,0) + rotation(1,1) + rotation(2,2) + 1);
        // transform.transform.rotation.w = a;
        // transform.transform.rotation.x = (rotation(2,1) - rotation(1,2))/(4*a);
        // transform.transform.rotation.y = (rotation(0,2) - rotation(2,0))/(4*a);
        // transform.transform.rotation.z = (rotation(1,0) - rotation(0,1))/(4*a);
        // std::cout<<"quaternion = ["<< transform.transform.rotation.w<<"|"
        //                             << transform.transform.rotation.x<<"|"
        //                             << transform.transform.rotation.y<<"|"
        //                             << transform.transform.rotation.z<<"]"<<std::endl;

        // Conversion matrice de rotation → quaternion (méthode robuste)
        double trace = rotation(0,0) + rotation(1,1) + rotation(2,2);
        double w, x, y, z;

        if (trace > 0) {
            double s = 0.5 / sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (rotation(2,1) - rotation(1,2)) * s;
            y = (rotation(0,2) - rotation(2,0)) * s;
            z = (rotation(1,0) - rotation(0,1)) * s;
        }
        else if (rotation(0,0) > rotation(1,1) && rotation(0,0) > rotation(2,2)) {
            double s = 2.0 * sqrt(1.0 + rotation(0,0) - rotation(1,1) - rotation(2,2));
            w = (rotation(2,1) - rotation(1,2)) / s;
            x = 0.25 * s;
            y = (rotation(0,1) + rotation(1,0)) / s;
            z = (rotation(0,2) + rotation(2,0)) / s;
        }
        else if (rotation(1,1) > rotation(2,2)) {
            double s = 2.0 * sqrt(1.0 + rotation(1,1) - rotation(0,0) - rotation(2,2));
            w = (rotation(0,2) - rotation(2,0)) / s;
            x = (rotation(0,1) + rotation(1,0)) / s;
            y = 0.25 * s;
            z = (rotation(1,2) + rotation(2,1)) / s;
        }
        else {
            double s = 2.0 * sqrt(1.0 + rotation(2,2) - rotation(0,0) - rotation(1,1));
            w = (rotation(1,0) - rotation(0,1)) / s;
            x = (rotation(0,2) + rotation(2,0)) / s;
            y = (rotation(1,2) + rotation(2,1)) / s;
            z = 0.25 * s;
        }

        return transform;
    }
        // a completer dans le fichier Transformation.cpp        
        // retourne l'inverse de la Transformation actuelle
        Transformation inverse();
               
        // ne pas changer
        // affichage par std::cout
        friend std::ostream & operator<< (std::ostream & output,
                        const Transformation & T)
        {
            output << "position = " << T.position.transpose() << std::endl;
            output << "rotation = " << T.rotation << std::endl;
            return output;
        }

        // variables de la classe.
        Eigen::Matrix<double,3,1> position;
        Eigen::Matrix<double,3,3> rotation;
};

Transformation operator* (const Transformation& A, const Transformation &B);

// renvoie les transformation de base
Transformation RotX(double q);
Transformation RotY(double q);
Transformation RotZ(double q);

#endif 



