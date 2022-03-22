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

#include <fstream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>
#include <tinyxml2.h>
#include <urdf_model/link.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_exception/exception.h>

namespace urdf
{
void parseMaterial(Material& material, tinyxml2::XMLElement* config, bool only_name_is_ok)
{
    bool has_rgb = false;
    bool has_filename = false;

    material.clear();

    if (!config->Attribute("name"))
        throw ParseError("Material must contain a name attribute");
    material.name = config->Attribute("name");

    // texture
    tinyxml2::XMLElement* t = config->FirstChildElement("texture");
    if (t)
    {
        if (t->Attribute("filename"))
        {
            material.texture_filename = t->Attribute("filename");
            has_filename = true;
        }
    }

    // color
    tinyxml2::XMLElement* c = config->FirstChildElement("color");
    if (c)
    {
        if (c->Attribute("rgba"))
        {
            try
            {
                material.color.init(c->Attribute("rgba"));
                has_rgb = true;
            }
            catch (const ParseError&)
            {
                material.color.clear();
            }
        }
    }

    if (!has_rgb && !has_filename)
    {
        if (!only_name_is_ok)    // no need for an error if only name is ok
        {
            if (!has_rgb)
                throw ParseError(std::string("Material [" + material.name + "] color has no rgba"));
            if (!has_filename)
                throw ParseError(std::string("Material [" + material.name + "] not defined in file"));
        }
    }
}

bool parseSphere(Sphere& s, tinyxml2::XMLElement* c)
{
    s.clear();

    s.type = Geometry::SPHERE;
    if (!c->Attribute("radius"))
    {
        return false;
    }

    try
    {
        s.radius = strToDouble(c->Attribute("radius"));
    }
    catch (std::runtime_error&)
    {
        std::stringstream stm;
        stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
        return false;
    }

    return true;
}

bool parseBox(Box& b, tinyxml2::XMLElement* c)
{
    b.clear();

    b.type = Geometry::BOX;
    if (!c->Attribute("size"))
    {
        return false;
    }
    try
    {
        b.dim.init(c->Attribute("size"));
    }
    catch (const ParseError&)
    {
        b.dim.clear();
        return false;
    }
    return true;
}

bool parseCylinder(Cylinder& y, tinyxml2::XMLElement* c)
{
    y.clear();

    y.type = Geometry::CYLINDER;
    if (!c->Attribute("length") || !c->Attribute("radius"))
    {
        return false;
    }

    try
    {
        y.length = strToDouble(c->Attribute("length"));
    }
    catch (std::runtime_error&)
    {
        std::stringstream stm;
        stm << "length [" << c->Attribute("length") << "] is not a valid float";
        return false;
    }

    try
    {
        y.radius = strToDouble(c->Attribute("radius"));
    }
    catch (std::runtime_error&)
    {
        std::stringstream stm;
        stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
        return false;
    }

    return true;
}

bool parseMesh(Mesh& m, tinyxml2::XMLElement* c)
{
    m.clear();

    m.type = Geometry::MESH;
    if (!c->Attribute("filename"))
    {
        return false;
    }

    m.filename = c->Attribute("filename");

    if (c->Attribute("scale"))
    {
        try
        {
            m.scale.init(c->Attribute("scale"));
        }
        catch (const ParseError&)
        {
            m.scale.clear();
            return false;
        }
    }
    else
    {
        m.scale.x = m.scale.y = m.scale.z = 1;
    }
    return true;
}

GeometrySharedPtr parseGeometry(tinyxml2::XMLElement* g)
{
    GeometrySharedPtr geom;
    if (!g)
        return geom;

    tinyxml2::XMLElement* shape = g->FirstChildElement();
    if (!shape)
    {
        return geom;
    }

    std::string type_name = shape->Name();
    if (type_name == "sphere")
    {
        auto* s = new Sphere();
        geom.reset(s);
        if (parseSphere(*s, shape))
            return geom;
    }
    else if (type_name == "box")
    {
        Box* b = new Box();
        geom.reset(b);
        if (parseBox(*b, shape))
            return geom;
    }
    else if (type_name == "cylinder")
    {
        auto* c = new Cylinder();
        geom.reset(c);
        if (parseCylinder(*c, shape))
            return geom;
    }
    else if (type_name == "mesh")
    {
        Mesh* m = new Mesh();
        geom.reset(m);
        if (parseMesh(*m, shape))
            return geom;
    }
    else
    {
        return geom;
    }

    return {};
}

bool parseInertial(Inertial& i, tinyxml2::XMLElement* config)
{
    i.clear();

    // Origin
    tinyxml2::XMLElement* o = config->FirstChildElement("origin");
    if (o)
    {
        parsePose(i.origin, o);
    }

    tinyxml2::XMLElement* mass_xml = config->FirstChildElement("mass");
    if (!mass_xml)
    {
        return false;
    }
    if (!mass_xml->Attribute("value"))
    {
        return false;
    }

    try
    {
        i.mass = strToDouble(mass_xml->Attribute("value"));
    }
    catch (std::runtime_error&)
    {
        std::stringstream stm;
        stm << "Inertial: mass [" << mass_xml->Attribute("value") << "] is not a float";
        return false;
    }

    tinyxml2::XMLElement* inertia_xml = config->FirstChildElement("inertia");
    if (!inertia_xml)
    {
        return false;
    }

    std::vector<std::pair<std::string, double>> attrs{ std::make_pair("ixx", 0.0), std::make_pair("ixy", 0.0),
                                                       std::make_pair("ixz", 0.0), std::make_pair("iyy", 0.0),
                                                       std::make_pair("iyz", 0.0), std::make_pair("izz", 0.0) };

    for (auto& attr : attrs)
    {
        if (!inertia_xml->Attribute(attr.first.c_str()))
        {
            std::stringstream stm;
            stm << "Inertial: inertia element missing " << attr.first << " attribute";
            return false;
        }

        try
        {
            attr.second = strToDouble(inertia_xml->Attribute(attr.first.c_str()));
        }
        catch (std::runtime_error&)
        {
            std::stringstream stm;
            stm << "Inertial: inertia element " << attr.first << " is not a valid double";
            return false;
        }
    }

    i.ixx = attrs[0].second;
    i.ixy = attrs[1].second;
    i.ixz = attrs[2].second;
    i.iyy = attrs[3].second;
    i.iyz = attrs[4].second;
    i.izz = attrs[5].second;

    return true;
}

bool parseVisual(Visual& vis, tinyxml2::XMLElement* config)
{
    vis.clear();

    // Origin
    tinyxml2::XMLElement* o = config->FirstChildElement("origin");
    if (o)
    {
        parsePose(vis.origin, o);
    }

    // Geometry
    tinyxml2::XMLElement* geom = config->FirstChildElement("geometry");
    vis.geometry = parseGeometry(geom);
    if (!vis.geometry)
        return false;

    const char* name_char = config->Attribute("name");
    if (name_char)
        vis.name = name_char;

    // Material
    tinyxml2::XMLElement* mat = config->FirstChildElement("material");
    if (mat)
    {
        // get material name
        if (!mat->Attribute("name"))
        {
            return false;
        }
        vis.material_name = mat->Attribute("name");

        // try to parse material element in place
        vis.material = std::make_shared<Material>();
        try
        {
            parseMaterial(*vis.material, mat, true);
        }
        catch (...)
        {
            vis.material.reset();
        }
    }

    return true;
}

bool parseCollision(Collision& col, tinyxml2::XMLElement* config)
{
    col.clear();

    // Origin
    tinyxml2::XMLElement* o = config->FirstChildElement("origin");
    if (o)
    {
        parsePose(col.origin, o);
    }

    // Geometry
    tinyxml2::XMLElement* geom = config->FirstChildElement("geometry");
    col.geometry = parseGeometry(geom);
    if (!col.geometry)
        return false;

    const char* name_char = config->Attribute("name");
    if (name_char)
        col.name = name_char;

    return true;
}

bool parseLink(Link& link, tinyxml2::XMLElement* config)
{
    link.clear();

    const char* name_char = config->Attribute("name");
    if (!name_char)
    {
        return false;
    }
    link.name = std::string(name_char);

    // Inertial (optional)
    tinyxml2::XMLElement* i = config->FirstChildElement("inertial");
    if (i)
    {
        link.inertial = std::make_shared<Inertial>();
        if (!parseInertial(*link.inertial, i))
        {
            return false;
        }
    }

    // Multiple Visuals (optional)
    for (tinyxml2::XMLElement* vis_xml = config->FirstChildElement("visual"); vis_xml;
         vis_xml = vis_xml->NextSiblingElement("visual"))
    {
        VisualSharedPtr vis;
        vis = std::make_shared<Visual>();
        if (parseVisual(*vis, vis_xml))
        {
            link.visual_array.push_back(vis);
        }
        else
        {
            vis.reset();
            return false;
        }
    }

    // Visual (optional)
    // Assign the first visual to the .visual ptr, if it exists
    if (!link.visual_array.empty())
        link.visual = link.visual_array[0];

    // Multiple Collisions (optional)
    for (tinyxml2::XMLElement* col_xml = config->FirstChildElement("collision"); col_xml;
         col_xml = col_xml->NextSiblingElement("collision"))
    {
        CollisionSharedPtr col;
        col = std::make_shared<Collision>();
        if (parseCollision(*col, col_xml))
        {
            link.collision_array.push_back(col);
        }
        else
        {
            col.reset();
            return false;
        }
    }

    // Collision (optional)
    // Assign the first collision to the .collision ptr, if it exists
    if (!link.collision_array.empty())
        link.collision = link.collision_array[0];

    return true;
}

/* exports */
bool exportPose(Pose& pose, tinyxml2::XMLElement* xml);

bool exportMaterial(Material& material, tinyxml2::XMLElement* xml)
{
    tinyxml2::XMLElement* material_xml = xml->GetDocument()->NewElement("material");
    material_xml->SetAttribute("name", material.name.c_str());

    tinyxml2::XMLElement* texture = xml->GetDocument()->NewElement("texture");
    if (!material.texture_filename.empty())
        texture->SetAttribute("filename", material.texture_filename.c_str());
    material_xml->LinkEndChild(texture);

    tinyxml2::XMLElement* color = xml->GetDocument()->NewElement("color");
    color->SetAttribute("rgba", urdf_export_helpers::values2str(material.color).c_str());
    material_xml->LinkEndChild(color);
    xml->LinkEndChild(material_xml);
    return true;
}

bool exportSphere(Sphere& s, tinyxml2::XMLElement* xml)
{
    // e.g. add <sphere radius="1"/>
    tinyxml2::XMLElement* sphere_xml = xml->GetDocument()->NewElement("sphere");
    sphere_xml->SetAttribute("radius", urdf_export_helpers::values2str(s.radius).c_str());
    xml->LinkEndChild(sphere_xml);
    return true;
}

bool exportBox(Box& b, tinyxml2::XMLElement* xml)
{
    // e.g. add <box size="1 1 1"/>
    tinyxml2::XMLElement* box_xml = xml->GetDocument()->NewElement("box");
    box_xml->SetAttribute("size", urdf_export_helpers::values2str(b.dim).c_str());
    xml->LinkEndChild(box_xml);
    return true;
}

bool exportCylinder(Cylinder& y, tinyxml2::XMLElement* xml)
{
    // e.g. add <cylinder radius="1"/>
    tinyxml2::XMLElement* cylinder_xml = xml->GetDocument()->NewElement("cylinder");
    cylinder_xml->SetAttribute("radius", urdf_export_helpers::values2str(y.radius).c_str());
    cylinder_xml->SetAttribute("length", urdf_export_helpers::values2str(y.length).c_str());
    xml->LinkEndChild(cylinder_xml);
    return true;
}

bool exportMesh(Mesh& m, tinyxml2::XMLElement* xml)
{
    // e.g. add <mesh filename="my_file" scale="1 1 1"/>
    tinyxml2::XMLElement* mesh_xml = xml->GetDocument()->NewElement("mesh");
    if (!m.filename.empty())
        mesh_xml->SetAttribute("filename", m.filename.c_str());
    mesh_xml->SetAttribute("scale", urdf_export_helpers::values2str(m.scale).c_str());
    xml->LinkEndChild(mesh_xml);
    return true;
}

bool exportGeometry(GeometrySharedPtr& geom, tinyxml2::XMLElement* xml)
{
    tinyxml2::XMLElement* geometry_xml = xml->GetDocument()->NewElement("geometry");
    if (urdf::dynamic_pointer_cast<Sphere>(geom))
    {
        exportSphere((*(urdf::dynamic_pointer_cast<Sphere>(geom))), geometry_xml);
    }
    else if (urdf::dynamic_pointer_cast<Box>(geom))
    {
        exportBox((*(urdf::dynamic_pointer_cast<Box>(geom))), geometry_xml);
    }
    else if (urdf::dynamic_pointer_cast<Cylinder>(geom))
    {
        exportCylinder((*(urdf::dynamic_pointer_cast<Cylinder>(geom))), geometry_xml);
    }
    else if (urdf::dynamic_pointer_cast<Mesh>(geom))
    {
        exportMesh((*(urdf::dynamic_pointer_cast<Mesh>(geom))), geometry_xml);
    }
    else
    {
        auto* s = new Sphere();
        s->radius = 0.03;
        geom.reset(s);
        exportSphere((*(urdf::dynamic_pointer_cast<Sphere>(geom))), geometry_xml);
    }

    xml->LinkEndChild(geometry_xml);
    return true;
}

bool exportInertial(Inertial& i, tinyxml2::XMLElement* xml)
{
    // adds <inertial>
    //        <mass value="1"/>
    //        <pose xyz="0 0 0" rpy="0 0 0"/>
    //        <inertia ixx="1" ixy="0" />
    //      </inertial>
    tinyxml2::XMLElement* inertial_xml = xml->GetDocument()->NewElement("inertial");

    tinyxml2::XMLElement* mass_xml = xml->GetDocument()->NewElement("mass");
    mass_xml->SetAttribute("value", urdf_export_helpers::values2str(i.mass).c_str());
    inertial_xml->LinkEndChild(mass_xml);

    exportPose(i.origin, inertial_xml);

    tinyxml2::XMLElement* inertia_xml = xml->GetDocument()->NewElement("inertia");
    inertia_xml->SetAttribute("ixx", urdf_export_helpers::values2str(i.ixx).c_str());
    inertia_xml->SetAttribute("ixy", urdf_export_helpers::values2str(i.ixy).c_str());
    inertia_xml->SetAttribute("ixz", urdf_export_helpers::values2str(i.ixz).c_str());
    inertia_xml->SetAttribute("iyy", urdf_export_helpers::values2str(i.iyy).c_str());
    inertia_xml->SetAttribute("iyz", urdf_export_helpers::values2str(i.iyz).c_str());
    inertia_xml->SetAttribute("izz", urdf_export_helpers::values2str(i.izz).c_str());
    inertial_xml->LinkEndChild(inertia_xml);

    xml->LinkEndChild(inertial_xml);

    return true;
}

bool exportVisual(Visual& vis, tinyxml2::XMLElement* xml)
{
    // <visual group="default">
    //   <origin rpy="0 0 0" xyz="0 0 0"/>
    //   <geometry>
    //     <mesh filename="mesh.dae"/>
    //   </geometry>
    //   <material name="Grey"/>
    // </visual>
    tinyxml2::XMLElement* visual_xml = xml->GetDocument()->NewElement("visual");

    exportPose(vis.origin, visual_xml);

    exportGeometry(vis.geometry, visual_xml);

    if (vis.material)
        exportMaterial(*vis.material, visual_xml);

    xml->LinkEndChild(visual_xml);

    return true;
}

bool exportCollision(Collision& col, tinyxml2::XMLElement* xml)
{
    // <collision group="default">
    //   <origin rpy="0 0 0" xyz="0 0 0"/>
    //   <geometry>
    //     <mesh filename="mesh.dae"/>
    //   </geometry>
    //   <material name="Grey"/>
    // </collision>
    tinyxml2::XMLElement* collision_xml = xml->GetDocument()->NewElement("collision");

    exportPose(col.origin, collision_xml);

    exportGeometry(col.geometry, collision_xml);

    xml->LinkEndChild(collision_xml);

    return true;
}

bool exportLink(Link& link, tinyxml2::XMLElement* xml)
{
    tinyxml2::XMLElement* link_xml = xml->GetDocument()->NewElement("link");
    link_xml->SetAttribute("name", link.name.c_str());

    if (link.inertial)
        exportInertial(*link.inertial, link_xml);
    for (auto & i : link.visual_array)
        exportVisual(*i, link_xml);
    for (auto & i : link.collision_array)
        exportCollision(*i, link_xml);

    xml->LinkEndChild(link_xml);

    return true;
}

}    // namespace urdf
