#include "Transformation.h"

// retourne la matrice de rotation en fonction des quaternions
Eigen::Matrix<double,3,3> setQuaternion( const double & a, const double & b, const double & c, const double &d)
{
    Eigen::Matrix<double,3,3> rotation;

    rotation(0,0) = a*a + b*b - c*c - d*d;
    rotation(0,1) = 2*(b*c - a*d);
    rotation(0,2) = 2*(b*d + a*c);

    rotation(1,0) = 2*(b*c + a*d);
    rotation(1,1) = a*a - b*b + c*c - d*d;
    rotation(1,2) = 2*(c*d - a*b);

    rotation(2,0) = 2*(b*d - a*c);
    rotation(2,1) = 2*(c*d + a*b);
    rotation(2,2) = a*a - b*b - c*c + d*d;
        
    return rotation;
}

// renvoie la matrice (out) de transformation inverse de la matrice présente (*this)
// actuellement on recopie this
Transformation Transformation::inverse()
{
    Transformation out;
    out.rotation = this->rotation.transpose();
    out.position = -(out.rotation * this->position);
    return out;
}

// renvoie la matrice (out) de transformation comme la multiplication de la transformation A et de la transformation B
// en exemple on recopie A
Transformation operator* (const Transformation& A, const Transformation &B)
{
    Transformation out;
    out.rotation = A.rotation * B.rotation;
    out.position = A.rotation * B.position + A.position;
    return out;
}

// retourne la matrice de rotation en fonction des angles roll, pitch, yaw
Eigen::Matrix<double,3,3> setRPY( const double & roll, const double & pitch, const double & yaw)
{
    Eigen::Matrix<double,3,3> rotation;

    const double cr = cos(roll); // ROLL
    const double sr = sin(roll);
    const double cp = cos(pitch); // PITCH
    const double sp = sin(pitch);
    const double cy = cos(yaw); // YAW
    const double sy = sin(yaw);

    rotation(0,0) = cy * cp;
    rotation(0,1) = cy * sp * sr - sy * cr;
    rotation(0,2) = cy * sp * cr + sy * sr;

    rotation(1,0) = sy * cp;
    rotation(1,1) = sy * sp * sr + cy * cr;
    rotation(1,2) = sy * sp * cr - cy * sr;

    rotation(2,0) = -sp;
    rotation(2,1) = cp * sr;
    rotation(2,2) = cp * cr;
    
    return rotation;
}

// retourne les angles roll, pitch et yaw à partir d'une matrice de rotation
Eigen::Matrix<double,3,1> getRPY(const Eigen::Matrix<double,3,3> mat)
{
    Eigen::Matrix<double,3,1> out;   
    out(1) = atan2(-mat(2,0), sqrt(mat(0,0)*mat(0,0) + mat(1,0)*mat(1,0)));
    out(0) = atan2(mat(2,1), mat(2,2));
    out(2) = atan2(mat(1,0), mat(0,0));
    
    return out;
}


// Renvoie la matrice de transformation pour une rotation autour de X
Transformation RotX(double q)
{
    double s = sin(q);
    double c = cos(q);
    Transformation out;
    out.rotation(0,0) = 1;
    out.rotation(0,1) = 0;
    out.rotation(0,2) = 0;
    out.rotation(1,0) = 0;
    out.rotation(1,1) = c;
    out.rotation(1,2) = -s;
    out.rotation(2,0) = 0;
    out.rotation(2,1) = s;
    out.rotation(2,2) = c;
    
    return out;
}

// Renvoie la matrice de transformation pour une rotation autour de Y
Transformation RotY(double q)
{
    double s = sin(q);
    double c = cos(q);    
    Transformation out;
   out.rotation(0,0) = c;
    out.rotation(0,1) = 0;
    out.rotation(0,2) = s;
    out.rotation(1,0) = 0;
    out.rotation(1,1) = 1;
    out.rotation(1,2) = 0;
    out.rotation(2,0) = -s;
    out.rotation(2,1) = 0;
    out.rotation(2,2) = c;
    
    return out;   
}

// Renvoie la matrice de transformation pour une rotation autour de Z
Transformation RotZ(double q)
{
    double s = sin(q);
    double c = cos(q);    
    Transformation out;
    out.rotation(0,0) = c;
    out.rotation(0,1) = -s;
    out.rotation(0,2) = 0;
    out.rotation(1,0) = s;
    out.rotation(1,1) = c;
    out.rotation(1,2) = 0;
    out.rotation(2,0) = 0;
    out.rotation(2,1) = 0;
    out.rotation(2,2) = 1;

    return out;    
}

