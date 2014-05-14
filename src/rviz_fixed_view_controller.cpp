/*
 * Copyright (c) 2009-2013, Willow Garage, Inc.
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
 *
 * author: Adam Leeper (adamleeper@gmail.com)
 *         David Gossow
 */

#include <stdint.h>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreViewport.h>

#include "rviz/display_context.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/editable_enum_property.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"

#include "rviz_fixed_view_controller/rviz_fixed_view_controller.h"

using namespace rviz;

namespace rviz_fixed_view_controller
{

// Pre-compute rotation matrices for the different view modes.
static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
  Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Y ) *
  Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Z );

static const Ogre::Quaternion OPTICAL_TO_CAMERA_ROTATION =
  Ogre::Quaternion( Ogre::Radian( Ogre::Math::PI ), Ogre::Vector3::UNIT_X );

// Strings for selecting view mode styles.
static const std::string MODE_ROBOT_FRAME = "X-forward, Z-up";
static const std::string MODE_CAMERA_FRAME = "Z-forward, X-right";

FixedViewController::FixedViewController()
{
  fov_property_ = new FloatProperty( "Vertical Field Of View", 50.0, "Vertical opening angle of the camera.", this );
  fov_property_->setMin( 5.0 );
  fov_property_->setMax( 130.0 );

  view_mode_property_ = new EditableEnumProperty(
              "View Mode", QString::fromStdString(MODE_ROBOT_FRAME),
              "Select the camera frame convention.", this);
  view_mode_property_->addOptionStd(MODE_ROBOT_FRAME);
  view_mode_property_->addOptionStd(MODE_CAMERA_FRAME);
  view_mode_property_->setStdString(MODE_ROBOT_FRAME);
}

void FixedViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();
  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
  camera_->setOrientation(ROBOT_TO_CAMERA_ROTATION);

  connect( view_mode_property_, SIGNAL( changed() ),
           this, SLOT( onViewModePropertyChanged() ), Qt::UniqueConnection);
}

FixedViewController::~FixedViewController()
{
}

void FixedViewController::onViewModePropertyChanged()
{
    if (view_mode_property_->getStdString() == MODE_ROBOT_FRAME)
        camera_->setOrientation(ROBOT_TO_CAMERA_ROTATION);
    else if (view_mode_property_->getStdString() == MODE_CAMERA_FRAME)
        camera_->setOrientation(OPTICAL_TO_CAMERA_ROTATION);
    else
        camera_->setOrientation(ROBOT_TO_CAMERA_ROTATION);
}

void FixedViewController::reset()
{
  FramePositionTrackingViewController::reset();
}

void FixedViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update( dt, ros_dt );
  camera_->setFOVy( Ogre::Degree(fov_property_->getFloat()) );
  target_scene_node_->setOrientation( reference_orientation_ );
}

void FixedViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  setStatus( "<b>Right-Click / Mouse Wheel:</b>: Zoom.  " );

  int32_t diff_x = event.x - event.last_x;
  int32_t diff_y = event.y - event.last_y;

  if( event.right() )
  {
    setCursor( Zoom );
    zoom( -diff_y * 0.1);
  }

  if( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;
    zoom( diff * 0.001 );
  }
}

void FixedViewController::zoom( float amount )
{
  fov_property_->setFloat( fov_property_->getFloat() * (1.0 + amount) );
}


}  // namespace rviz_fixed_view_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_fixed_view_controller::FixedViewController, rviz::ViewController )
