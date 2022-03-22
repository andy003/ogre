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

#include <memory>

#define URDF_TYPEDEF_CLASS_POINTER(Class)                                                                              \
    class Class;                                                                                                       \
    typedef std::shared_ptr<Class> Class##SharedPtr;                                                                   \
    typedef std::shared_ptr<const Class> Class##ConstSharedPtr;                                                        \
    typedef std::weak_ptr<Class> Class##WeakPtr

namespace urdf
{

// shared pointer used in joint.h
typedef std::shared_ptr<double> DoubleSharedPtr;

URDF_TYPEDEF_CLASS_POINTER(Box);
URDF_TYPEDEF_CLASS_POINTER(Collision);
URDF_TYPEDEF_CLASS_POINTER(Cylinder);
URDF_TYPEDEF_CLASS_POINTER(Geometry);
URDF_TYPEDEF_CLASS_POINTER(Inertial);
URDF_TYPEDEF_CLASS_POINTER(Joint);
URDF_TYPEDEF_CLASS_POINTER(JointCalibration);
URDF_TYPEDEF_CLASS_POINTER(JointDynamics);
URDF_TYPEDEF_CLASS_POINTER(JointLimits);
URDF_TYPEDEF_CLASS_POINTER(JointMimic);
URDF_TYPEDEF_CLASS_POINTER(JointSafety);
URDF_TYPEDEF_CLASS_POINTER(Link);
URDF_TYPEDEF_CLASS_POINTER(Material);
URDF_TYPEDEF_CLASS_POINTER(Mesh);
URDF_TYPEDEF_CLASS_POINTER(Sphere);
URDF_TYPEDEF_CLASS_POINTER(Visual);

// create *_pointer_cast functions in urdf namespace
template <class T, class U>
std::shared_ptr<T> const_pointer_cast(std::shared_ptr<U> const& r)
{
    return std::const_pointer_cast<T>(r);
}

template <class T, class U>
std::shared_ptr<T> dynamic_pointer_cast(std::shared_ptr<U> const& r)
{
    return std::dynamic_pointer_cast<T>(r);
}

template <class T, class U>
std::shared_ptr<T> static_pointer_cast(std::shared_ptr<U> const& r)
{
    return std::static_pointer_cast<T>(r);
}

}    // namespace urdf
