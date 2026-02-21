
#include "Robot.h"


Robot::Robot()
{
    // A MODIFIER
//  uint nb_joint = 1;      // définition du nombre d'articulation
//  uint nb_body = 1;       // définition du nombre de corps
//     TJoint.resize(nb_joint);   // on définit que le vecteur TJoint (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)
//     TLink.resize(nb_body);     // on définit que le vecteur TLink  (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)    
//     TStatic.resize(nb_joint);  // on définit que le vecteur TStatic (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)
    
//     qmin.resize(nb_joint);  // on définit un vecteur de double avec nb_joint composant (en radian)
//     qmax.resize(nb_joint);  // on définit un vecteur de double avec nb_joint composant (en radian)
    
//     TStatic[0] = Transformation (0,0,0,0,0,0);                // on définit la premiere transformation, les 6 paramètres sont : Tx, Ty, Tz, Roll, Pitch, Yaw  (angle en degré)
    
    
    
    TJoint.resize(6);   // on définit que le vecteur TJoint (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)
    TLink.resize(7);    // on définit que le vecteur TLink  (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)    
    
    // on initialise des variables
    TStatic.push_back( Transformation (0,0,0,0,0,0));                // on rajoute une transformation au vecteur TStatic
    // A COMPLETER 
    TStatic.push_back( Transformation (0,-0.002,0.072,-90.0,0,0));  // braccio_arm_base_link to shoulder_link
    TStatic.push_back( Transformation (0,0,0.125,-90.0,0,0));  // shoulder_link to elbow_link
    TStatic.push_back( Transformation (0,0,0.125,-90.0,0,0));  // elbow_link to wrist_pitch_link
    TStatic.push_back( Transformation (0,0,0.06,0,0,90.0));  // wrist_pitch_link to wrist_roll_link
    TStatic.push_back( Transformation (0.0,0.01,0.12,0,180,0));  // wrist_roll_link to gripper
    
    qmin.push_back(0);      qmax.push_back(PI);          //  on ajoute une valeur au vecteur qmin et qmax
    // A COMPLETER 
    qmin.push_back(0.2618);      qmax.push_back(2.8798);          // shoulder_joint
    qmin.push_back(0);      qmax.push_back(PI);      // elbow_joint
    qmin.push_back(0);      qmax.push_back(PI);  // wrist_roll_joint
    qmin.push_back(0);      qmax.push_back(PI); // wrist_roll_joint
    qmin.push_back(0.175);      qmax.push_back(1.2741); // gripper_joint    
}

// renvoi la pose de l'effecteur en fonction des variables articulaires Q.
Transformation Robot::ModGeoDirect( const VECTOR& Q)
{
    double q1 = Q.data[0];
    double q2 = Q.data[1];
    double q3 = Q.data[2];
    double q4 = Q.data[3];
    double q5 = Q.data[4];
    
    // A COMPLETER
//     Transformation Teff;
    
//     return Teff;
    
    
    TJoint[0] = RotZ(q1);
    TJoint[1] = RotX(q2);
    TJoint[2] = RotX(q3);
    TJoint[3] = RotX(q4);
    TJoint[4] = RotZ(-q5);
    TJoint[5] = Transformation();   // identité
    
    for (int i=0;i<6;i++)
        TLink[i+1] = TLink[i] * TStatic[i] * TJoint[i];
       
    return TLink[6];
}

// Cette fonction calcule les valeurs articulaires pour que l'effecteur atteigne le point définit par le vecteur pos.
// Si le point n'est pas accessible, la fonction renvoie false
bool  Robot::ModGeoInverse( const Eigen::Matrix<double,3,1> & pos,
                            VECTOR & Q)
{    
    Q.data.resize(6);
    double q1,q2,q3,q4,q5;
    
    // A COMPLETER
    
    double X = pos(0);
    double Y = pos(1);
    double Z = pos(2);
    
    Transformation T;
    T.position = pos;

    
    double l1 = 0.125;
    double l2 = 0.125;

    // position du centre du repere du poignet
    Transformation TWrist = T * TStatic[5].inverse()  * TStatic[4].inverse();

    // premiere hypothese a mettre a jour
    double x= TWrist.position(0);
    double y= TWrist.position(1);
    double z= TWrist.position(2); // si la pince est verticale  

    //  calcul de q1
    double dxy = sqrt( x*x+y*y);
    q1= atan2(y/dxy,x/dxy);
    Transformation TShoulder = TStatic[0] * RotZ(q1-PI/2) * TStatic[1];
    Transformation STW = TShoulder.inverse() * TWrist;
    double D = STW.position.norm();

    // l'orientation selon l'axe vertival theta_v et selon l'axe du bras : theta_h    
    double theta_v = atan2(STW.rotation(0,1),STW.rotation(0,0));    
    double theta_h =  atan2(-STW.rotation(1,2),STW.rotation(2,2)) + PI/2;
    
    while(theta_v < PI)
        theta_v += PI;
    while(theta_v > PI)
        theta_v -= PI;
    
    while(theta_h < 0)
        theta_h += PI/2;
    while(theta_h > PI/2)
        theta_h -= PI/2;    
    
    
    if(D> l1+l2)
    {
        std::cout<<"ERRROR POINT TROP LOIN"<<std::endl;
        std::cout<<"D = "<<D<<std::endl;
        return false;
    }
    else
    {
        double cos3 = (l1*l1+l2*l2-D*D) / (2*l1*l2);
        q3 = acos(cos3);
        double alpha = (l1*l1+D*D-l2*l2) / (2*l1*D);
        q2 = (asin ( -(STW.position(1))/D) + acos(alpha));
        q4 = theta_h -q2-q3 + PI;    
        q5 = theta_v ;
    }

    
    // robot offset 
    q1 -= 1.5708;
    q3 -= 1.5708;
    
    while (q1 < -PI)    q1 += 2*PI;
    while (q1 > PI)    q1 -= 2*PI;
    
    while (q3 < -PI)    q3 += 2*PI;
    while (q3 > PI)    q3 -= 2*PI;
    
    while (q5 < 0)  q5 += PI;
    while (q5 > PI)  q5 -= PI;
    
    Q.data[0] = q1;
    Q.data[1] = q2;
    Q.data[2] = q3;
    Q.data[3] = q4;
    Q.data[4] = q5;

    
    // ne pas oublier de vérifier que les valeurs sont dans les limites articulaires
    for (int i=0;i<5;i++)   // on ne vérifie pas le gripper
    {
        if (Q.data[i] < qmin[i]) Q.data[i] = qmin[i];
        if (Q.data[i] > qmax[i]) Q.data[i] = qmax[i];
    }    
    
    return true;
}

// Renvoie la jacobienne (position en 3D d'un point de l'effecteur) pour un vecteur articulaire
Eigen::Matrix<double,3,5> Robot::ComputeJacobian( const VECTOR& Q)
{
    Eigen::Matrix<double,3,5> out;
    double delta = 1e-3;
    Transformation InitPose=ModGeoDirect(Q);
    Transformation Pose1,Pose2;
    std_msgs::msg::Float64MultiArray Qtmp;  // ROS2: ajout de ::msg
    for (int i=0;i<5;i++)
    {
        Qtmp = Q;
        Qtmp.data[i] = Q.data[i] + delta;
        Pose1 = ModGeoDirect(Qtmp);
        Qtmp.data[i] = Q.data[i] - delta;
        Pose2 = ModGeoDirect(Qtmp);
        
        for (int j=0;j<3;j++)
        {
            out(j,i) = (Pose1.position(j) - Pose2.position(j))/(2*delta);
        }
    }
    
    return out;
}



// Calcule le vecteur articulaire suivant permettant de se rapprocher de la cible
// entrees
//      Qin : Vecteur articulaire initial
//      CurrentPosition : Position (vecteur 3D) actuelle de l'effecteur (par modèle ou par mesure)
//      DesiredPosition : Position (vecteur 3D) désirée de l'effecteur
// sorties
//      Qout : Vecteur articulaire permettant de se rapprocher de la cible
//      retourne la distance entre la position actuelle et désirée
double Robot::ComputeControl(   const VECTOR& Qin,
                                const Eigen::Matrix<double,3,1>& CurrentPosition,
                                const Eigen::Matrix<double,3,1>& DesiredPosition,
                                VECTOR& Qout)
{
    Eigen::Matrix<double,3,5> Jacobian = ComputeJacobian(Qin);
    Eigen::Matrix<double,5,1> DeltaQ =  Jacobian.transpose() * (  Jacobian * Jacobian.transpose() ).inverse() * (DesiredPosition-CurrentPosition);
    // std::cout<<"Jacobian = "<< Jacobian<<std::endl;
    // std::cout<<"CurrentPosition = "<< CurrentPosition.transpose()<<std::endl;
    // std::cout<<"DesiredPosition = "<< DesiredPosition.transpose()<<std::endl;
    // std::cout<<"DeltaQ = "<< DeltaQ.transpose()<<std::endl;
    for (int i=0;i<5;i++)
        Qout.data[i] = Qin.data[i] + DeltaQ(i)*0.1;    

    for (int i=0;i<5;i++)   // on ne vérifie pas le gripper
    {
        if (Qout.data[i] < qmin[i])
        {
            // std::cout<<"Violation de Q("<<i<<") min"<<std::endl;
            Qout.data[i] = qmin[i];
            
        }
        if (Qout.data[i] > qmax[i])
        {
            // std::cout<<"Violation de Q("<<i<<") max"<<std::endl;
            Qout.data[i] = qmax[i];
            
        }
    }        
    
    return (DesiredPosition-CurrentPosition).norm();
}
