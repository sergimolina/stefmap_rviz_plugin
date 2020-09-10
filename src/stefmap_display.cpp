/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of stefmap_rviz_plugin.
 *
 *   stefmap_rviz_plugin is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   stefmap_rviz_plugin is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with stefmap_rviz_plugin.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#include "stefmap_display.h"

namespace stefmap_rviz_plugin {

STeFMapDisplay::STeFMapDisplay() {
  color_property_ = new rviz::ColorProperty(
      "Color", QColor(204, 51, 204), "Color to draw the acceleration arrows.",
      this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
  arrow_size_property_ =
      new rviz::FloatProperty("Size", 1.0, "Choose a value between 0.0 and 1.0",
                              this, SLOT(updateArrowSize()));
}

void STeFMapDisplay::onInitialize() { MFDClass::onInitialize(); }

// Clear the visuals by deleting their objects.
void STeFMapDisplay::reset() { MFDClass::reset(); }

void STeFMapDisplay::updateArrowSize() {
  visual_->setArrowSize(arrow_size_property_->getFloat());
}

// Set the current color and alpha values for each visual.
void STeFMapDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual_->setColor(color.r, color.g, color.b, alpha);
}

void STeFMapDisplay::processMessage(
    const stefmap_ros::STeFMapMsg::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  boost::shared_ptr<STeFMapVisual> visual;
  visual.reset(new STeFMapVisual(context_->getSceneManager(), scene_node_));

  // Now set or update the contents of the chosen visual.
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visual_ = visual;
}

} /* namespace stefmap_rviz_plugin */

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stefmap_rviz_plugin::STeFMapDisplay, rviz::Display)
