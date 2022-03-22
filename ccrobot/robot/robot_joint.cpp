/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include "robot_joint.h"
#include "robot_link.h"
#include "robot.h"

#include <OgreSceneNode.h>

#include <ogre_helper/arrow.h>
#include <ogre_helper/axes.h>

namespace rviz
{
RobotJoint::RobotJoint(Robot* robot, const urdf::JointConstSharedPtr& joint)
    : robot_(robot)
    , name_(joint->name)
    , parent_link_name_(joint->parent_link_name)
    , child_link_name_(joint->child_link_name)
    , has_decendent_links_with_geometry_(true)
    , doing_set_checkbox_(false)
    , axes_(nullptr)
    , axis_(nullptr)
{
    std::string type;
    if (joint->type == urdf::Joint::UNKNOWN)
        type = "unknown";
    else if (joint->type == urdf::Joint::REVOLUTE)
        type = "revolute";
    else if (joint->type == urdf::Joint::CONTINUOUS)
        type = "continuous";
    else if (joint->type == urdf::Joint::PRISMATIC)
        type = "prismatic";
    else if (joint->type == urdf::Joint::FLOATING)
        type = "floating";
    else if (joint->type == urdf::Joint::PLANAR)
        type = "planar";
    else if (joint->type == urdf::Joint::FIXED)
        type = "fixed";

    const urdf::Vector3& pos = joint->parent_to_joint_origin_transform.position;
    const urdf::Rotation& rot = joint->parent_to_joint_origin_transform.rotation;
    joint_origin_pos_ = Ogre::Vector3(pos.x, pos.y, pos.z);
    joint_origin_rot_ = Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z);
}

RobotJoint::~RobotJoint()
{
    delete axes_;
    delete axis_;
}

RobotJoint* RobotJoint::getParentJoint()
{
    RobotLink* parent_link = robot_->getLink(parent_link_name_);
    if (!parent_link)
        return nullptr;

    const std::string& parent_joint_name = parent_link->getParentJointName();
    if (parent_joint_name.empty())
        return nullptr;

    return robot_->getJoint(parent_joint_name);
}

void RobotJoint::calculateJointCheckboxesRecursive(int& links_with_geom, int& links_with_geom_checked, // NOLINT(misc-no-recursion)
                                                   int& links_with_geom_unchecked)
{
    links_with_geom_checked = 0;
    links_with_geom_unchecked = 0;

    RobotLink* link = robot_->getLink(child_link_name_);
    if (link && link->hasGeometry())
    {
        bool checked = link->getLinkProperty()->getValue().toBool();
        links_with_geom_checked += checked ? 1 : 0;
        links_with_geom_unchecked += checked ? 0 : 1;
    }
    links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

    auto child_joint_it = link->getChildJointNames().begin();
    auto child_joint_end = link->getChildJointNames().end();
    for (; child_joint_it != child_joint_end; ++child_joint_it)
    {
        RobotJoint* child_joint = robot_->getJoint(*child_joint_it);
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
}

void RobotJoint::getChildLinkState(int& links_with_geom, int& links_with_geom_checked, int& links_with_geom_unchecked, // NOLINT(misc-no-recursion)
                                   bool recursive) const
{
    links_with_geom_checked = 0;
    links_with_geom_unchecked = 0;

    RobotLink* link = robot_->getLink(child_link_name_);
    if (link && link->hasGeometry())
    {
        bool checked = link->getLinkProperty()->getValue().toBool();
        links_with_geom_checked += checked ? 1 : 0;
        links_with_geom_unchecked += checked ? 0 : 1;
    }

    if (recursive)
    {
        auto child_joint_it = link->getChildJointNames().begin();
        auto child_joint_end = link->getChildJointNames().end();
        for (; child_joint_it != child_joint_end; ++child_joint_it)
        {
            RobotJoint* child_joint = robot_->getJoint(*child_joint_it);
            if (child_joint)
            {
                int child_links_with_geom;
                int child_links_with_geom_checked;
                int child_links_with_geom_unchecked;
                child_joint->getChildLinkState(child_links_with_geom, child_links_with_geom_checked,
                                               child_links_with_geom_unchecked, recursive);
                links_with_geom_checked += child_links_with_geom_checked;
                links_with_geom_unchecked += child_links_with_geom_unchecked;
            }
        }
    }

    links_with_geom = links_with_geom_checked + links_with_geom_unchecked;
}

void RobotJoint::updateChildVisibility()
{
    if (doing_set_checkbox_)
        return;

    if (!hasDescendentLinksWithGeometry())
        return;

    bool visible = getEnabled();

    RobotLink* link = robot_->getLink(child_link_name_);
    if (link)
    {
        if (link->hasGeometry())
        {
            link->getLinkProperty()->setValue(visible);
        }

        if (styleIsTree())
        {
            auto child_joint_it = link->getChildJointNames().begin();
            auto child_joint_end = link->getChildJointNames().end();
            for (; child_joint_it != child_joint_end; ++child_joint_it)
            {
                RobotJoint* child_joint = robot_->getJoint(*child_joint_it);
                if (child_joint)
                {
                    child_joint->getJointProperty()->setValue(visible);
                }
            }
        }
    }
}

void RobotJoint::updateAxes()
{
    if (axes_property_->getValue().toBool())
    {
        if (!axes_)
        {
            static int count = 0;
            std::stringstream ss;
            ss << "Axes for joint " << name_ << count++;
            axes_ = new Axes(robot_->getSceneManager(), robot_->getOtherNode(), 0.1, 0.01);
            axes_->getSceneNode()->setVisible(getEnabled());

            axes_->setPosition(position_property_->getVector());
            axes_->setOrientation(orientation_property_->getQuaternion());
        }
    }
    else
    {
        if (axes_)
        {
            delete axes_;
            axes_ = nullptr;
        }
    }
}

void RobotJoint::updateAxis()
{
    if (show_axis_property_->getValue().toBool())
    {
        if (!axis_)
        {
            static int count = 0;
            std::stringstream ss;
            ss << "Axis for joint " << name_ << count++;
            axis_ = new Arrow(robot_->getSceneManager(), robot_->getOtherNode(), 0.15, 0.05, 0.05, 0.08);
            axis_->getSceneNode()->setVisible(getEnabled());

            axis_->setPosition(position_property_->getVector());
            axis_->setOrientation(orientation_property_->getQuaternion());

            // TODO(lucasw) store an Ogre::ColorValue and set it according to
            // joint type.
            axis_->setColor(0.0, 0.8, 0.0, 1.0);
        }
    }
    else
    {
        if (axis_)
        {
            delete axis_;
            axis_ = nullptr;
        }
    }
}

void RobotJoint::setTransforms(const Ogre::Vector3& parent_link_position,
                               const Ogre::Quaternion& parent_link_orientation)
{
    Ogre::Vector3 position = parent_link_position + parent_link_orientation * joint_origin_pos_;
    Ogre::Quaternion orientation = parent_link_orientation * joint_origin_rot_;

    if (axes_)
    {
        axes_->setPosition(position);
        axes_->setOrientation(orientation);
    }
    if (axis_)
    {
        axis_->setPosition(position);
        axis_->setOrientation(orientation);
//        axis_->setDirection(parent_link_orientation * axis_property_->getVector());
    }
}

}    // namespace rviz
