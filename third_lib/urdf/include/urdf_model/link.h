/**
 ***********************************************************************************************************************
 *
 * @author  ZhangRan
 * @version 1.0.0
 *
 * <h2><center>&copy; COPYRIGHT 2022 </center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <string>
#include <vector>
#include <map>

#include "joint.h"
#include "color.h"
#include "types.h"

namespace urdf
{

class Geometry
{
public:
    enum
    {
        SPHERE,
        BOX,
        CYLINDER,
        MESH
    } type;

    virtual ~Geometry(void)
    {
    }
};

class Sphere : public Geometry
{
public:
    Sphere()
    {
        this->clear();
        type = SPHERE;
    };
    double radius;

    void clear()
    {
        radius = 0;
    };
};

class Box : public Geometry
{
public:
    Box()
    {
        this->clear();
        type = BOX;
    };
    Vector3 dim;

    void clear()
    {
        this->dim.clear();
    };
};

class Cylinder : public Geometry
{
public:
    Cylinder()
    {
        this->clear();
        type = CYLINDER;
    };
    double length;
    double radius;

    void clear()
    {
        length = 0;
        radius = 0;
    };
};

class Mesh : public Geometry
{
public:
    Mesh()
    {
        this->clear();
        type = MESH;
    };
    std::string filename;
    Vector3 scale;

    void clear()
    {
        filename.clear();
        // default scale
        scale.x = 1;
        scale.y = 1;
        scale.z = 1;
    };
};

class Material
{
public:
    Material()
    {
        this->clear();
    };
    std::string name;
    std::string texture_filename;
    Color color;

    void clear()
    {
        color.clear();
        texture_filename.clear();
        name.clear();
    };
};

class Inertial
{
public:
    Inertial()
    {
        this->clear();
    };
    Pose origin;
    double mass;
    double ixx, ixy, ixz, iyy, iyz, izz;

    void clear()
    {
        origin.clear();
        mass = 0;
        ixx = ixy = ixz = iyy = iyz = izz = 0;
    };
};

class Visual
{
public:
    Visual()
    {
        this->clear();
    };
    Pose origin;
    GeometrySharedPtr geometry;

    std::string material_name;
    MaterialSharedPtr material;

    void clear()
    {
        origin.clear();
        material_name.clear();
        material.reset();
        geometry.reset();
        name.clear();
    };

    std::string name;
};

class Collision
{
public:
    Collision()
    {
        this->clear();
    };
    Pose origin;
    GeometrySharedPtr geometry;

    void clear()
    {
        origin.clear();
        geometry.reset();
        name.clear();
    };

    std::string name;
};

class Link
{
public:
    Link()
    {
        this->clear();
    };

    std::string name;

    /// inertial element
    InertialSharedPtr inertial;

    /// visual element
    VisualSharedPtr visual;

    /// collision element
    CollisionSharedPtr collision;

    /// if more than one collision element is specified, all collision elements are placed in this array (the collision
    /// member points to the first element of the array)
    std::vector<CollisionSharedPtr> collision_array;

    /// if more than one visual element is specified, all visual elements are placed in this array (the visual member
    /// points to the first element of the array)
    std::vector<VisualSharedPtr> visual_array;

    /// Parent Joint element
    ///   explicitly stating "parent" because we want directional-ness for tree structure
    ///   every link can have one parent
    JointSharedPtr parent_joint;

    std::vector<JointSharedPtr> child_joints;
    std::vector<LinkSharedPtr> child_links;

    LinkSharedPtr getParent() const
    {
        return parent_link_.lock();
    };

    void setParent(const LinkSharedPtr& parent)
    {
        parent_link_ = parent;
    }

    void clear()
    {
        this->name.clear();
        this->inertial.reset();
        this->visual.reset();
        this->collision.reset();
        this->parent_joint.reset();
        this->child_joints.clear();
        this->child_links.clear();
        this->collision_array.clear();
        this->visual_array.clear();
    };

private:
    LinkWeakPtr parent_link_;
};
}    // namespace urdf
