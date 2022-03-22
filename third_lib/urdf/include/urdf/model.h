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
#include <memory>
#include <tinyxml2.h>
#include <urdf_model/model.h>

namespace urdf
{

typedef std::shared_ptr<ModelInterface> ModelInterfaceSharedPtr;

class Model : public ModelInterface
{
public:
    bool initXml(const tinyxml2::XMLElement* xml);

    bool initXml(const tinyxml2::XMLDocument* xml);

    bool initFile(const std::string& filename);

    bool initString(const std::string& xmlstring);
};

}    // namespace urdf
