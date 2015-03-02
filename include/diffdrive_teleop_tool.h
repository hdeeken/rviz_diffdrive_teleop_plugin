/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2014 University of Osnabrück
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES  (INCLUDING,  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE   OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * teleop_tool.h
 *
 *  Author: Henning Deeken <hdeeken@uos.de>
 *
 */

#ifndef RVIZ_DIFFDRIVE_TELEOP_TOOL_H
#define RVIZ_DIFFDRIVE_TELEOP_TOOL_H

#include <map>
#include <vector>

#include <QIcon>
#include <QMessageBox>
#include <QApplication>

#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>
#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/load_resource.h>

#include <rviz/default_plugin/tools/move_tool.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 *@class DiffDriveTeleopTool
 *
 *@brief Implements a rviz tool that allows to navigate a wheeled robot using the keyboard.
 */

namespace rviz
{

class DiffDriveTeleopTool: public rviz::Tool
{
Q_OBJECT

public:
  DiffDriveTeleopTool();
  ~DiffDriveTeleopTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );
  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);

private Q_SLOTS:

  void setWalkSpeed(){ walk_speed_ = (double) walk_speed_property_->getFloat(); }
  void setWalkBoost()
  {
    if(walk_boost_property_->getFloat() < 0.0)
    {
      walk_boost_ = 0.0;
    }
    else if(walk_boost_property_->getFloat() > 1.0)
    {
      walk_boost_ = 1.0;
    }
    else
    {
      walk_boost_ = (double) walk_boost_property_->getFloat();
    }
  }

  void setYawSpeed(){ yaw_speed_ = (double) yaw_speed_property_->getFloat(); }
  void setYawBoost()
  {
    if(yaw_boost_property_->getFloat() < 0.0)
    {
      yaw_boost_ = 0.0;
    }
    else if(yaw_boost_property_->getFloat() > 1.0)
    {
      yaw_boost_ = 1.0;
    }
    else
    {
      yaw_boost_ = (double) yaw_boost_property_->getFloat();
    }
  }

  void setLeftHandMode(){ left_hand_mode_ = left_hand_property_->getBool(); }

private:
  ros::NodeHandle node_;
  ros::Publisher command_publisher_;

  bool left_hand_mode_;

  double walk_speed_;
  double walk_boost_;
  double yaw_speed_;
  double yaw_boost_;

  FloatProperty* walk_speed_property_;
  FloatProperty* walk_boost_property_;
  FloatProperty* yaw_speed_property_;
  FloatProperty* yaw_boost_property_;
  BoolProperty* left_hand_property_;

  MoveTool move_tool_;
};
} // end namespace rviz
#endif
