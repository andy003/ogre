/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot.h"
#include "robot_link.h"
#include "robot_joint.h"

#include "ogre_helpers/object.h"
#include "ogre_helpers/shape.h"
#include "ogre_helpers/axes.h"

#include "urdf_model/model.h"

#include "OgreSceneNode.h"
#include "OgreSceneManager.h"
#include "OgreEntity.h"
#include "OgreMaterialManager.h"
#include "OgreMaterial.h"
#include "OgreResourceGroupManager.h"

#include <utility>

namespace rviz
{
Robot::Robot(Ogre::SceneNode* root_node, std::string name)
    : scene_manager_(nullptr)
    , visible_(true)
    , visual_visible_(true)
    , collision_visible_(false)
    , doing_set_checkbox_(false)
    , robot_loaded_(false)
    , inChangedEnableAllLinks(false)
    , name_(std::move(name))
{
    root_visual_node_ = root_node->createChildSceneNode();
    root_collision_node_ = root_node->createChildSceneNode();
    root_other_node_ = root_node->createChildSceneNode();

    link_factory_ = new LinkFactory();

    setVisualVisible(visual_visible_);
    setCollisionVisible(collision_visible_);
    setAlpha(1.0f);
}

Robot::~Robot()
{
    clear();

    scene_manager_->destroySceneNode(root_visual_node_->getName());
    scene_manager_->destroySceneNode(root_collision_node_->getName());
    scene_manager_->destroySceneNode(root_other_node_->getName());
    delete link_factory_;
    delete link_tree_;
}

void Robot::setLinkFactory(LinkFactory* link_factory)
{
    if (link_factory)
    {
        delete link_factory_;
        link_factory_ = link_factory;
    }
}

void Robot::setVisible(bool visible)
{
    visible_ = visible;
    if (visible)
    {
        root_visual_node_->setVisible(visual_visible_);
        root_collision_node_->setVisible(collision_visible_);
        updateLinkVisibilities();
    }
    else
    {
        root_visual_node_->setVisible(false);
        root_collision_node_->setVisible(false);
        updateLinkVisibilities();
    }
}

void Robot::setVisualVisible(bool visible)
{
    visual_visible_ = visible;
    updateLinkVisibilities();
}

void Robot::setCollisionVisible(bool visible)
{
    collision_visible_ = visible;
    updateLinkVisibilities();
}

void Robot::updateLinkVisibilities()
{
    auto it = links_.begin();
    auto end = links_.end();
    for (; it != end; ++it)
    {
        RobotLink* link = it->second;
        link->updateVisibility();
    }
}

bool Robot::isVisible() const
{
    return visible_;
}

bool Robot::isVisualVisible() const
{
    return visual_visible_;
}

bool Robot::isCollisionVisible() const
{
    return collision_visible_;
}

void Robot::setAlpha(float a)
{
    alpha_ = a;

    auto it = links_.begin();
    auto end = links_.end();
    for (; it != end; ++it)
    {
        RobotLink* link = it->second;

        link->setRobotAlpha(alpha_);
    }
}

void Robot::clear()
{
    // unparent all link and joint properties so they can be deleted in arbitrary
    // order without being delete by their parent propeties (which vary based on
    // style)
    unparentLinkProperties();

    auto link_it = links_.begin();
    auto link_end = links_.end();
    for (; link_it != link_end; ++link_it)
    {
        RobotLink* link = link_it->second;
        delete link;
    }

    auto joint_it = joints_.begin();
    auto joint_end = joints_.end();
    for (; joint_it != joint_end; ++joint_it)
    {
        RobotJoint* joint = joint_it->second;
        delete joint;
    }

    links_.clear();
    joints_.clear();
    root_visual_node_->removeAndDestroyAllChildren();
    root_collision_node_->removeAndDestroyAllChildren();
    root_other_node_->removeAndDestroyAllChildren();
}

RobotLink* Robot::LinkFactory::createLink(Robot* robot, const urdf::LinkConstSharedPtr& link,
                                          const std::string& parent_joint_name, bool visual, bool collision)
{
    return new RobotLink(robot, link, parent_joint_name, visual, collision);
}

RobotJoint* Robot::LinkFactory::createJoint(Robot* robot, const urdf::JointConstSharedPtr& joint)
{
    return new RobotJoint(robot, joint);
}

void Robot::load(const urdf::ModelInterface& urdf, bool visual, bool collision)
{
    robot_loaded_ = false;

    // clear out any data (properties, shapes, etc) from a previously loaded robot.
    clear();

    // the root link is discovered below.  Set to NULL until found.
    root_link_ = nullptr;

    // Create properties for each link.
    // Properties are not added to display until changedLinkTreeStyle() is called (below).
    {
        typedef std::map<std::string, urdf::LinkSharedPtr> M_NameToUrdfLink;
        auto link_it = urdf.links_.begin();
        auto link_end = urdf.links_.end();
        for (; link_it != link_end; ++link_it)
        {
            const urdf::LinkConstSharedPtr& urdf_link = link_it->second;
            std::string parent_joint_name;

            if (urdf_link != urdf.getRoot() && urdf_link->parent_joint)
            {
                parent_joint_name = urdf_link->parent_joint->name;
            }

            RobotLink* link = link_factory_->createLink(this, urdf_link, parent_joint_name, visual, collision);

            if (urdf_link == urdf.getRoot())
            {
                root_link_ = link;
            }

            links_[urdf_link->name] = link;

            link->setRobotAlpha(alpha_);
        }
    }

    // Create properties for each joint.
    // Properties are not added to display until changedLinkTreeStyle() is called (below).
    {
        typedef std::map<std::string, urdf::JointSharedPtr> M_NameToUrdfJoint;
        auto joint_it = urdf.joints_.begin();
        auto joint_end = urdf.joints_.end();
        for (; joint_it != joint_end; ++joint_it)
        {
            const urdf::JointConstSharedPtr& urdf_joint = joint_it->second;
            RobotJoint* joint = link_factory_->createJoint(this, urdf_joint);

            joints_[urdf_joint->name] = joint;

            joint->setRobotAlpha(alpha_);
        }
    }

    // robot is now loaded
    robot_loaded_ = true;

    setVisualVisible(isVisualVisible());
    setCollisionVisible(isCollisionVisible());
}

void Robot::unparentLinkProperties()
{
    // remove link properties from their parents
    auto link_it = links_.begin();
    auto link_end = links_.end();
    for (; link_it != link_end; ++link_it)
    {
        link_it->second->setParentProperty(nullptr);
    }

    // remove joint properties from their parents
    auto joint_it = joints_.begin();
    auto joint_end = joints_.end();
    for (; joint_it != joint_end; ++joint_it)
    {
        joint_it->second->setParentProperty(nullptr);
    }
}

void Robot::useDetailProperty(bool use_detail)
{
    // remove sub properties and add them to detail
    auto link_it = links_.begin();
    auto link_end = links_.end();
    for (; link_it != link_end; ++link_it)
    {
        link_it->second->useDetailProperty(use_detail);
    }

    // remove joint properties from their parents
    auto joint_it = joints_.begin();
    auto joint_end = joints_.end();
    for (; joint_it != joint_end; ++joint_it)
    {
        joint_it->second->useDetailProperty(use_detail);
    }
}

bool Robot::styleShowLink(LinkTreeStyle style)
{
    return style == STYLE_LINK_LIST || style == STYLE_LINK_TREE || style == STYLE_JOINT_LINK_TREE;
}

bool Robot::styleShowJoint(LinkTreeStyle style)
{
    return style == STYLE_JOINT_LIST || style == STYLE_JOINT_LINK_TREE;
}

bool Robot::styleIsTree(LinkTreeStyle style)
{
    return style == STYLE_LINK_TREE || style == STYLE_JOINT_LINK_TREE;
}

// insert properties into link_tree_ according to style
void Robot::changedLinkTreeStyle()
{
    if (!robot_loaded_)
        return;

    LinkTreeStyle style = LinkTreeStyle(link_tree_style_->getOptionInt());

    unparentLinkProperties();

    // expand_tree_->setValue(false);

    switch (style)
    {
        case STYLE_LINK_TREE:
        case STYLE_JOINT_LINK_TREE:
            useDetailProperty(true);
            if (root_link_)
            {
                addLinkToLinkTree(style, link_tree_, root_link_);
            }
            break;

        case STYLE_JOINT_LIST: {
            useDetailProperty(false);
            auto joint_it = joints_.begin();
            auto joint_end = joints_.end();
            for (; joint_it != joint_end; ++joint_it)
            {
                joint_it->second->setParentProperty(link_tree_);
                joint_it->second->setJointPropertyDescription();
            }
            break;
        }

        case STYLE_LINK_LIST:
        default:
            useDetailProperty(false);
            auto link_it = links_.begin();
            auto link_end = links_.end();
            for (; link_it != link_end; ++link_it)
            {
                link_it->second->setParentProperty(link_tree_);
            }
            break;
    }

    switch (style)
    {
        case STYLE_LINK_TREE:
            link_tree_->setName("Link Tree");
            link_tree_->setDescription("A tree of all links in the robot.  Uncheck a link to hide its geometry.");
            expand_tree_->show();
            expand_link_details_->show();
            expand_joint_details_->hide();
            break;

        case STYLE_JOINT_LINK_TREE:
            link_tree_->setName("Link/Joint Tree");
            link_tree_->setDescription("A tree of all joints and links in the robot.  Uncheck a link to hide its "
                                       "geometry.");
            expand_tree_->show();
            expand_link_details_->show();
            expand_joint_details_->show();
            break;

        case STYLE_JOINT_LIST:
            link_tree_->setName("Joints");
            link_tree_->setDescription("All joints in the robot in alphabetic order.");
            expand_tree_->hide();
            expand_link_details_->hide();
            expand_joint_details_->show();
            break;

        case STYLE_LINK_LIST:
        default:
            link_tree_->setName("Links");
            link_tree_->setDescription("All links in the robot in alphabetic order.  Uncheck a link to hide its "
                                       "geometry.");
            expand_tree_->hide();
            expand_link_details_->show();
            expand_joint_details_->hide();
            break;
    }

    expand_link_details_->setValue(false);
    expand_joint_details_->setValue(false);
    expand_tree_->setValue(false);
    calculateJointCheckboxes();
}

// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addLinkToLinkTree(LinkTreeStyle style, Property* parent, RobotLink* link)
{
    if (styleShowLink(style))
    {
        link->setParentProperty(parent);
        parent = link->getLinkProperty();
    }

    std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
    std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
    for (; child_joint_it != child_joint_end; ++child_joint_it)
    {
        RobotJoint* child_joint = getJoint(*child_joint_it);
        if (child_joint)
        {
            addJointToLinkTree(style, parent, child_joint);
        }
    }
}

// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addJointToLinkTree(LinkTreeStyle style, Property* parent, RobotJoint* joint)
{
    if (styleShowJoint(style))
    {
        joint->setParentProperty(parent);
        parent = joint->getJointProperty();
        joint->setJointPropertyDescription();
    }

    RobotLink* link = getLink(joint->getChildLinkName());
    if (link)
    {
        addLinkToLinkTree(style, parent, link);
    }
}

RobotLink* Robot::getLink(const std::string& name)
{
    auto it = links_.find(name);
    if (it == links_.end())
    {
        ROS_WARN("Link [%s] does not exist", name.c_str());
        return nullptr;
    }

    return it->second;
}

RobotJoint* Robot::getJoint(const std::string& name)
{
    auto it = joints_.find(name);
    if (it == joints_.end())
    {
        ROS_WARN("Joint [%s] does not exist", name.c_str());
        return nullptr;
    }

    return it->second;
}

void Robot::calculateJointCheckboxes()
{
    if (inChangedEnableAllLinks || !robot_loaded_)
        return;

    int links_with_geom_checked = 0;
    int links_with_geom_unchecked = 0;

    // check root link
    RobotLink* link = root_link_;

    if (!link)
    {
        setEnableAllLinksCheckbox(QVariant());
        return;
    }

    if (link->hasGeometry())
    {
        bool checked = link->getLinkProperty()->getValue().toBool();
        links_with_geom_checked += checked ? 1 : 0;
        links_with_geom_unchecked += checked ? 0 : 1;
    }
    int links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

    // check all child links and joints recursively
    std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
    std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
    for (; child_joint_it != child_joint_end; ++child_joint_it)
    {
        RobotJoint* child_joint = getJoint(*child_joint_it);
        if (child_joint)
        {
            int child_links_with_geom;
            int child_links_with_geom_checked;
            int child_links_with_geom_unchecked;
            child_joint->calculateJointCheckboxesRecursive(child_links_with_geom, child_links_with_geom_checked,
                                                           child_links_with_geom_unchecked);
            links_with_geom_checked += child_links_with_geom_checked;
            links_with_geom_unchecked += child_links_with_geom_unchecked;
        }
    }
    links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

    if (!links_with_geom)
    {
        setEnableAllLinksCheckbox(QVariant());
    }
    else
    {
        setEnableAllLinksCheckbox(links_with_geom_unchecked == 0);
    }
}

void Robot::update(const LinkUpdater& updater)
{
    auto link_it = links_.begin();
    auto link_end = links_.end();
    for (; link_it != link_end; ++link_it)
    {
        RobotLink* link = link_it->second;

        link->setToNormalMaterial();

        Ogre::Vector3 visual_position, collision_position;
        Ogre::Quaternion visual_orientation, collision_orientation;
        if (updater.getLinkTransforms(link->getName(), visual_position, visual_orientation, collision_position,
                                      collision_orientation))
        {
            // Check if visual_orientation, visual_position, collision_orientation, and collision_position are
            // NaN.
            if (visual_orientation.isNaN())
            {
                ROS_ERROR_THROTTLE(1.0,
                                   "visual orientation of %s contains NaNs. "
                                   "Skipping render as long as the orientation is invalid.",
                                   link->getName().c_str());
                continue;
            }
            if (visual_position.isNaN())
            {
                ROS_ERROR_THROTTLE(1.0,
                                   "visual position of %s contains NaNs. Skipping render as long as the position is "
                                   "invalid.",
                                   link->getName().c_str());
                continue;
            }
            if (collision_orientation.isNaN())
            {
                ROS_ERROR_THROTTLE(1.0,
                                   "collision orientation of %s contains NaNs. "
                                   "Skipping render as long as the orientation is invalid.",
                                   link->getName().c_str());
                continue;
            }
            if (collision_position.isNaN())
            {
                ROS_ERROR_THROTTLE(1.0,
                                   "collision position of %s contains NaNs. "
                                   "Skipping render as long as the position is invalid.",
                                   link->getName().c_str());
                continue;
            }
            link->setTransforms(visual_position, visual_orientation, collision_position, collision_orientation);

            std::vector<std::string>::const_iterator joint_it = link->getChildJointNames().begin();
            std::vector<std::string>::const_iterator joint_end = link->getChildJointNames().end();
            for (; joint_it != joint_end; ++joint_it)
            {
                RobotJoint* joint = getJoint(*joint_it);
                if (joint)
                {
                    joint->setTransforms(visual_position, visual_orientation);
                }
            }
        }
        else
        {
            link->setToErrorMaterial();
        }
    }
}

void Robot::setPosition(const Ogre::Vector3& position)
{
    root_visual_node_->setPosition(position);
    root_collision_node_->setPosition(position);
}

void Robot::setOrientation(const Ogre::Quaternion& orientation)
{
    root_visual_node_->setOrientation(orientation);
    root_collision_node_->setOrientation(orientation);
}

void Robot::setScale(const Ogre::Vector3& scale)
{
    root_visual_node_->setScale(scale);
    root_collision_node_->setScale(scale);
}

const Ogre::Vector3& Robot::getPosition()
{
    return root_visual_node_->getPosition();
}

const Ogre::Quaternion& Robot::getOrientation()
{
    return root_visual_node_->getOrientation();
}

}    // namespace rviz
