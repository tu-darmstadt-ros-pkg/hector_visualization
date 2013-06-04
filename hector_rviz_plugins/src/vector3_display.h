/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef HECTOR_RVIZ_PLUGINS_VECTOR3_DISPLAY_H
#define HECTOR_RVIZ_PLUGINS_VECTOR3_DISPLAY_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include <geometry_msgs/Vector3Stamped.h>

#include <rviz/display.h>

namespace rviz
{
class Arrow;
class ColorProperty;
class FloatProperty;
class StringProperty;
class IntProperty;
class RosTopicProperty;
class TfFrameProperty;
}

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace hector_rviz_plugins
{

using namespace rviz;

class Vector3Display: public rviz::Display
{
Q_OBJECT
public:
  Vector3Display();
  virtual ~Vector3Display();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void updateColor();
  void updateTopic();
  void updateOriginFrame();
  void updateScale();
  void updateLength();

private:
  void subscribe();
  void unsubscribe();
  void clear();

  void incomingMessage( const geometry_msgs::Vector3Stamped::ConstPtr& message );
  void transformArrow( const geometry_msgs::Vector3Stamped::ConstPtr& message, Arrow* arrow );

  typedef std::deque<Arrow*> D_Arrow;
  D_Arrow arrows_;

  uint32_t messages_received_;

  boost::shared_ptr<Ogre::Vector3> last_position_;
  boost::shared_ptr<Ogre::Quaternion> last_orientation_;
  
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_;
  tf::MessageFilter<geometry_msgs::Vector3Stamped>* tf_filter_;

  ColorProperty* color_property_;
  RosTopicProperty* topic_property_;
  TfFrameProperty* origin_frame_property_;
  FloatProperty* scale_property_;
  FloatProperty* position_tolerance_property_;
  FloatProperty* angle_tolerance_property_;
  IntProperty* keep_property_;
};

} // end namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_VECTOR3_DISPLAY_H
