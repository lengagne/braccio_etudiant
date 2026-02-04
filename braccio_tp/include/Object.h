#ifndef __OBJECT_H__
#define __OBJECT_H__

#include "Transformation.h"

typedef struct marker
{
    unsigned int id;
    Transformation pose;
    bool known_reference;   // use to define if the value of the transformation is fixed or must be measured.
    bool already_seen;
} marker;

class Object
{
public:
    Object(const std::string& n)
    {
        name = n;
        ids.clear();
    }

    void add_marker( const marker& m)
    {
        relative_marker.push_back(m);
        ids.push_back(m.id);
    }

    bool IsDefined() const
    {
        return pose_defined;
    }

    std::vector<unsigned int> GetIds() const
    {
        return ids;
    }

    Transformation GetGlobalPose( ) const
    {
        return global_pose;
    }

    Transformation GetLocalPose(const unsigned int id) const
    {
        return relative_marker[id].pose;
    }

    std::string GetName() const
    {
        return name;
    }

    void ReInitPoseDefined()
    {
        pose_defined = false;
    }

    void SetGlobalPose( const Transformation & T)
    {
        global_pose = T;
        pose_defined = true;
    }

private:

    bool pose_defined = false;

    Transformation global_pose;             // pose of the object regarding the world frame.

    std::string name;
    std::vector<unsigned int > ids;
    std::vector<marker> relative_marker;    // define the pose of the marker regarding the object frame.

};

#endif
