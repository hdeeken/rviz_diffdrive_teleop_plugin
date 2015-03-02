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
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * teleop_tool.cpp
 *
 * Author: Henning Deeken {hdeeken@uos.de}
 *
 * Robot icon by Jean-Philippe Cabaroc
 *
 */

#include <teleop_tool.h>

namespace rviz
{

TeleopTool::TeleopTool()
{
  shortcut_key_ = 't';
  access_all_keys_ = true;

  command_publisher_ =  node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

TeleopTool::~TeleopTool() {}

void TeleopTool::onInitialize()
{
  setName( "Drive" );

  walk_speed_property_ = new FloatProperty( "Walk Speed", 1.0,
                                                            "The speed which is used for moving back and forth.",
                                                            getPropertyContainer(), SLOT( setWalkSpeed() ), this );

  walk_boost_property_ = new FloatProperty( "Walk Boost", 0.5,
                                                            "Gives the boost factor which is applied if pressing shift.",
                                                            getPropertyContainer(), SLOT( setWalkBoost() ), this );

  yaw_speed_property_ = new FloatProperty( "Yaw Speed", 1.5,
                                                            "The speed which is used for turning left and right.",
                                                            getPropertyContainer(), SLOT( setYawSpeed() ), this );

  yaw_boost_property_ = new FloatProperty( "Yaw Boost", 0.5,
                                                            "Gives the boost factor which is applied if pressing shift.",
                                                            getPropertyContainer(), SLOT( setYawBoost() ), this );


  left_hand_property_ = new BoolProperty( "Left Hand Mode", false,
                                                            "In left hand mode one uses the arrows to move around, instead of wasd.",
                                                            getPropertyContainer(), SLOT( setLeftHandMode() ), this );

  walk_speed_ = 1.0;
  walk_boost_ = 0.5;
  yaw_speed_ = 1.5;
  yaw_boost_ = 0.5;
  left_hand_mode_ = false;

  move_tool_.initialize( context_ );
}

void TeleopTool::activate(){ }

void TeleopTool::deactivate(){ }



int TeleopTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel* panel)
{

  ROS_INFO("Key: %s / %d", event->text().toStdString().c_str(), event->key());
  geometry_msgs::Twist command;
  
  double walk_update = walk_speed_;
  double yaw_update = yaw_speed_;

  if(event->modifiers() & Qt::ShiftModifier)
  {
    walk_update += walk_speed_ * walk_boost_;
    yaw_update += yaw_speed_ * yaw_boost_;
  }

  // move forward / backward
  if ((event->key() == Qt::Key_W && !left_hand_mode_) || (event->key() == Qt::Key_Up && left_hand_mode_))
  {
    ROS_INFO("Move Forward");
    command.linear.x = walk_update;
  }
 if ((event->key() == Qt::Key_S))
 {
   ROS_INFO("Move Backwardzzz");
 }
  if ((event->key() == Qt::Key_S && !left_hand_mode_))  //|| (event->key() == Qt::Key_Down && left_hand_mode_))
  {
    ROS_INFO("Move Backward");
    //command.linear.x = -walk_update;
  }

  // turn left / right
  if ((event->key() == Qt::Key_A && !left_hand_mode_) || (event->key() == Qt::Key_Left && left_hand_mode_))
  {
    ROS_INFO("Turn Left");
    command.angular.z = yaw_update;
  }

  if ((event->key() == Qt::Key_D && !left_hand_mode_) || (event->key() == Qt::Key_Right && left_hand_mode_))
  {
    ROS_INFO("Turn Right");
    command.angular.z = -yaw_update;
  }

  command_publisher_.publish(command);

  return Render;
}

int TeleopTool::processMouseEvent( ViewportMouseEvent& event )
{
  return move_tool_.processMouseEvent( event );
}


} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::TeleopTool, rviz::Tool)
