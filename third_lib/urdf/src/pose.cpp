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

#include <urdf_model/pose.h>
#include <fstream>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_exception/exception.h>

namespace urdf_export_helpers
{

std::string values2str(unsigned int count, const double* values, double (*conv)(double))
{
    std::stringstream ss;
    for (unsigned int i = 0; i < count; i++)
    {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}

std::string values2str(urdf::Vector3 vec)
{
    double xyz[3];
    xyz[0] = vec.x;
    xyz[1] = vec.y;
    xyz[2] = vec.z;
    return values2str(3, xyz);
}
std::string values2str(urdf::Rotation rot)
{
    double rpy[3];
    rot.getRPY(rpy[0], rpy[1], rpy[2]);
    return values2str(3, rpy);
}
std::string values2str(urdf::Color c)
{
    double rgba[4];
    rgba[0] = c.r;
    rgba[1] = c.g;
    rgba[2] = c.b;
    rgba[3] = c.a;
    return values2str(4, rgba);
}
std::string values2str(double d)
{
    return values2str(1, &d);
}
}    // namespace urdf_export_helpers

namespace urdf
{

void parsePose(Pose& pose, tinyxml2::XMLElement* xml)
{
    pose.clear();
    if (xml)
    {
        const char* xyz_str = xml->Attribute("xyz");
        if (xyz_str != NULL)
            pose.position.init(xyz_str);

        const char* rpy_str = xml->Attribute("rpy");
        if (rpy_str != NULL)
            pose.rotation.init(rpy_str);
    }
}

bool exportPose(Pose& pose, tinyxml2::XMLElement* xml)
{
    tinyxml2::XMLElement* origin = xml->GetDocument()->NewElement("origin");
    std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
    std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
    origin->SetAttribute("xyz", pose_xyz_str.c_str());
    origin->SetAttribute("rpy", pose_rpy_str.c_str());
    xml->LinkEndChild(origin);
    return true;
}

}    // namespace urdf
