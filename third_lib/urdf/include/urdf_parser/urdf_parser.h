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

#include <tinyxml2.h>
#include <urdf_model/model.h>
#include <urdf_model/color.h>
#include <urdf_model/utils.h>
#include <urdf_world/types.h>
#include <urdf_sensor/sensor.h>
#include <urdf_model_state/model_state.h>

namespace urdf_export_helpers
{

std::string values2str(unsigned int count, const double* values, double (*conv)(double) = nullptr);
std::string values2str(urdf::Vector3 vec);
std::string values2str(urdf::Rotation rot);
std::string values2str(urdf::Color c);
std::string values2str(double d);

// This lives here (rather than in model.cpp) so we can run tests on it.
class URDFVersion final
{
public:
    explicit URDFVersion(const char* attr)
    {
        // If the passed in attribute is NULL, it means it wasn't specified in the
        // XML, so we just assume version 1.0.
        if (attr == nullptr)
        {
            major_ = 1;
            minor_ = 0;
            return;
        }

        // We only accept version strings of the type <major>.<minor>
        std::vector<std::string> split;
        urdf::split_string(split, std::string(attr), ".");
        if (split.size() == 2)
        {
            major_ = strToUnsigned(split[0].c_str());
            minor_ = strToUnsigned(split[1].c_str());
        }
        else
        {
            throw std::runtime_error("The version attribute should be in the form 'x.y'");
        }
    }

    bool equal(uint32_t maj, uint32_t min) const
    {
        return this->major_ == maj && this->minor_ == min;
    }

    uint32_t getMajor() const
    {
        return major_;
    }

    uint32_t getMinor() const
    {
        return minor_;
    }

private:
    static uint32_t strToUnsigned(const char* str)
    {
        if (str[0] == '\0')
        {
            // This would get caught below, but we can make a nicer error message
            throw std::runtime_error("One of the fields of the version attribute is blank");
        }
        char* end = const_cast<char*>(str);
        long value = strtol(str, &end, 10);
        if (end == str)
        {
            // If the pointer didn't move at all, then we couldn't convert any of
            // the string to an integer.
            throw std::runtime_error("Version attribute is not an integer");
        }
        if (*end != '\0')
        {
            // Here, we didn't go all the way to the end of the string, which
            // means there was junk at the end
            throw std::runtime_error("Extra characters after the version number");
        }
        if (value < 0)
        {
            throw std::runtime_error("Version number must be positive");
        }

        return value;
    }

    uint32_t major_;
    uint32_t minor_;
};

}    // namespace urdf_export_helpers

namespace urdf
{
ModelInterfaceSharedPtr parseURDF(const std::string& xml_string);
ModelInterfaceSharedPtr parseURDFFile(const std::string& path);
tinyxml2::XMLDocument* exportURDF(ModelInterfaceSharedPtr& model);
tinyxml2::XMLDocument* exportURDF(const ModelInterface& model);
void parsePose(Pose&, tinyxml2::XMLElement*);
bool parseCamera(Camera&, tinyxml2::XMLElement*);
bool parseRay(Ray&, tinyxml2::XMLElement*);
bool parseSensor(Sensor&, tinyxml2::XMLElement*);
bool parseModelState(ModelState&, tinyxml2::XMLElement*);
}    // namespace urdf
