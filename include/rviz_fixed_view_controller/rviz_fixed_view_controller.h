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

#ifndef RVIZ_ORBIT_VIEW_CONTROLLER_H
#define RVIZ_ORBIT_VIEW_CONTROLLER_H

#include <QCursor>

#include "rviz/frame_position_tracking_view_controller.h"

namespace rviz
{
class FloatProperty;
class EditableEnumProperty;
}

namespace rviz_fixed_view_controller
{

class FixedViewController: public rviz::FramePositionTrackingViewController
{
Q_OBJECT
public:
  FixedViewController();
  virtual ~FixedViewController();

  virtual void onInitialize();

  virtual void reset();

  virtual void update(float dt, float ros_dt);

  virtual void handleMouseEvent(rviz::ViewportMouseEvent& event);

  void zoom( float amount );

protected Q_SLOTS:
  virtual void onViewModePropertyChanged();

private:

  rviz::FloatProperty* fov_property_;
  rviz::EditableEnumProperty* view_mode_property_; ///< Select between interpretations of "forward"

  // TODO could hook up the fov to a subscriber.

};

}  // namespace rviz_fixed_view_controller

#endif // RVIZ_VIEW_CONTROLLER_H
