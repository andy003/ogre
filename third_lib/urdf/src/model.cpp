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

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "urdf/model.h"
#include "tinyxml2.h"

#include <mutex>
#include <urdf_parser/urdf_parser.h>

namespace urdf
{
bool parseJoint(Joint& joint, tinyxml2::XMLElement* config);

bool Model::initFile(const std::string& filename)
{
    // get the entire file
    std::string xml_string;
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
        while (xml_file.good())
        {
            std::string line;
            std::getline(xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        return Model::initString(xml_string);
    }
    else
    {
        return false;
    }
}

bool Model::initXml(const tinyxml2::XMLDocument* xml_doc)
{
    if (!xml_doc)
    {
        return false;
    }

    tinyxml2::XMLPrinter printer;
    xml_doc->Print(&printer);
    std::string str(printer.CStr());

    return Model::initString(str);
}

bool Model::initXml(const tinyxml2::XMLElement* robot_xml)
{
    if (!robot_xml)
    {
        return false;
    }

    std::stringstream ss;
    tinyxml2::XMLPrinter printer;
    robot_xml->Accept(&printer);
    ss << printer.CStr();

    return Model::initString(ss.str());
}

extern bool parseLink(Link& link, tinyxml2::XMLElement* config);

ModelInterfaceSharedPtr parseURDF(const std::string& xml_string)
{
    ModelInterfaceSharedPtr model(new ModelInterface);
    model->clear();

    tinyxml2::XMLDocument xml_doc;
    xml_doc.Parse(xml_string.c_str());
    if (xml_doc.Error())
    {
        xml_doc.ClearError();
        model.reset();
        return model;
    }

    tinyxml2::XMLElement* robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml)
    {
        model.reset();
        return model;
    }

    // Get robot name
    const char* name = robot_xml->Attribute("name");
    if (!name)
    {
        model.reset();
        return model;
    }
    model->name_ = std::string(name);

    urdf_export_helpers::URDFVersion version(robot_xml->Attribute("version"));
    if (!version.equal(1, 0))
        throw std::runtime_error("Invalid 'version' specified; only version 1.0 is currently supported");

    // Get all Material elements
    for (tinyxml2::XMLElement* material_xml = robot_xml->FirstChildElement("material"); material_xml;
         material_xml = material_xml->NextSiblingElement("material"))
    {
        MaterialSharedPtr material;
        material = std::make_shared<Material>();

        try
        {
            //            parseMaterial(*material, material_xml, false);    // material needs to be fully defined here
            if (model->getMaterial(material->name))
            {
                material.reset();
                model.reset();
                return model;
            }
            else
            {
                model->materials_.insert(make_pair(material->name, material));
            }
        }
        catch (ParseError& /*e*/)
        {
            material.reset();
            model.reset();
            return model;
        }
    }

    // Get all Link elements
    for (tinyxml2::XMLElement* link_xml = robot_xml->FirstChildElement("link"); link_xml;
         link_xml = link_xml->NextSiblingElement("link"))
    {
        LinkSharedPtr link;
        link = std::make_shared<Link>();

        try
        {
            parseLink(*link, link_xml);
            if (model->getLink(link->name))
            {
                model.reset();
                return model;
            }
            else
            {
                // set link visual(s) material
                if (link->visual)
                {
                    //                    assignMaterial(link->visual, model, link->name.c_str());
                }
                for (const auto& visual : link->visual_array)
                {
                    //                    assignMaterial(visual, model, link->name.c_str());
                }

                model->links_.insert(make_pair(link->name, link));
            }
        }
        catch (ParseError& /*e*/)
        {
            model.reset();
            return model;
        }
    }
    if (model->links_.empty())
    {
        model.reset();
        return model;
    }

    // Get all Joint elements
    for (tinyxml2::XMLElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml;
         joint_xml = joint_xml->NextSiblingElement("joint"))
    {
        JointSharedPtr joint;
        joint = std::make_shared<Joint>();

        if (parseJoint(*joint, joint_xml))
        {
            if (model->getJoint(joint->name))
            {
                model.reset();
                return model;
            }
            else
            {
                model->joints_.insert(make_pair(joint->name, joint));
            }
        }
        else
        {
            model.reset();
            return model;
        }
    }

    // every link has children links and joints, but no parents, so we create a
    // local convenience data structure for keeping child->parent relations
    std::map<std::string, std::string> parent_link_tree;
    parent_link_tree.clear();

#if 0
    // building tree: name mapping
    try
    {
        model->initTree(parent_link_tree);
    }
    catch (ParseError& e)
    {
        model.reset();
        return model;
    }

    // find the root link
    try
    {
        model->initRoot(parent_link_tree);
    }
    catch (ParseError& e)
    {
        model.reset();
        return model;
    }
#endif

    return model;
}

bool Model::initString(const std::string& xml_string)
{
    urdf::ModelInterfaceSharedPtr model;

    model = parseURDF(xml_string);

    // copy data from model into this object
    if (model)
    {
        this->links_ = model->links_;
        this->joints_ = model->joints_;
        this->materials_ = model->materials_;
        this->name_ = model->name_;
        this->root_link_ = model->root_link_;
        return true;
    }

    return false;
}
}    // namespace urdf
