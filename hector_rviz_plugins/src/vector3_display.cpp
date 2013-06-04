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


#include <boost/bind.hpp>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "vector3_display.h"

namespace hector_rviz_plugins
{

Vector3Display::Vector3Display()
  : Display()
  , messages_received_( 0 )
{
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<geometry_msgs::Vector3Stamped>() ),
                                          "geometry_msgs::Vector3Stamped topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  color_property_ = new ColorProperty( "Color", QColor( 0, 25, 255 ),
                                       "Color of the arrows.",
                                       this, SLOT( updateColor() ));

  origin_frame_property_ = new TfFrameProperty( "Origin Frame", "base_link",
                                                "Frame that defines the origin of the vector.",
                                                this,
                                                0, true,
                                                SLOT( updateOriginFrame() ));

  position_tolerance_property_ = new FloatProperty( "Position Tolerance", .1,
                                                    "Distance, in meters from the last arrow dropped, "
                                                    "that will cause a new arrow to drop.",
                                                    this );
  position_tolerance_property_->setMin( 0 );
                                                
  angle_tolerance_property_ = new FloatProperty( "Angle Tolerance", .1,
                                                 "Angular distance from the last arrow dropped, "
                                                 "that will cause a new arrow to drop.",
                                                 this );
  angle_tolerance_property_->setMin( 0 );

  scale_property_ = new FloatProperty( "Scale", 1.0,
                                        "Scale of each arrow.",
                                        this, SLOT( updateScale() ));

  keep_property_ = new IntProperty( "Keep", 100,
                                    "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
                                    this );
  keep_property_->setMin( 0 );
}

Vector3Display::~Vector3Display()
{
  unsubscribe();
  clear();
  delete tf_filter_;
}

void Vector3Display::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<geometry_msgs::Vector3Stamped>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                          5, update_nh_ );

  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &Vector3Display::incomingMessage, this, _1 ));
  context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );

  origin_frame_property_->setFrameManager(context_->getFrameManager());
}

// Clear the visuals by deleting their objects.
void Vector3Display::clear()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  arrows_.clear();

  last_position_.reset();
  last_orientation_.reset();

  tf_filter_->clear();

  messages_received_ = 0;
  setStatus( StatusProperty::Warn, "Topic", "No messages received" );
}

void Vector3Display::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
  context_->queueRender();
}

void Vector3Display::updateColor()
{
  QColor color = color_property_->getColor();
  float red   = color.redF();
  float green = color.greenF();
  float blue  = color.blueF();

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for( ; it != end; ++it )
  {
    Arrow* arrow = *it;
    arrow->setColor( red, green, blue, 1.0f );
  }
  context_->queueRender();
}

void Vector3Display::updateScale()
{

}

void Vector3Display::updateOriginFrame()
{

}

void Vector3Display::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 5 );
    setStatus( StatusProperty::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
  }
}

void Vector3Display::unsubscribe()
{
  sub_.unsubscribe();
}

void Vector3Display::onEnable()
{
  subscribe();
}

void Vector3Display::onDisable()
{
  unsubscribe();
  clear();
}

// This is our callback to handle an incoming message.
void Vector3Display::incomingMessage( const geometry_msgs::Vector3Stamped::ConstPtr& message )
{
  ++messages_received_;

  if( !validateFloats( message->vector ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  setStatus( StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Vector3 message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation, fake_orientation;
  Ogre::Vector3 position, fake_position;
  if( !context_->getFrameManager()->getTransform( origin_frame_property_->getStdString(),
                                                  message->header.stamp,
                                                  position, fake_orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               qPrintable( origin_frame_property_->getString() ), qPrintable( fixed_frame_ ) );
    return;
  }

  if( !context_->getFrameManager()->getTransform( message->header.frame_id,
                                                  message->header.stamp,
                                                  fake_position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               message->header.frame_id.c_str(), qPrintable( fixed_frame_ ) );
    return;
  }

  Ogre::Vector3 vector( message->vector.x, message->vector.y, -message->vector.z );
  orientation = orientation * vector.getRotationTo(Ogre::Vector3::UNIT_Z);

  if( last_position_ && last_orientation_ )
  {
    if( (*last_position_ - position).length() < position_tolerance_property_->getFloat() &&
        (*last_orientation_ - orientation).normalise() < angle_tolerance_property_->getFloat() )
    {
      return;
    }
  }

  float length = vector.length();
  Arrow* arrow = new Arrow( scene_manager_, scene_node_, std::max(length - 0.2f, 0.0f), 0.05f, 0.2f, 0.2f );

  arrow->setPosition(position);
  arrow->setOrientation(orientation);

  QColor color = color_property_->getColor();
  arrow->setColor( color.redF(), color.greenF(), color.blueF(), 1.0f );

  Ogre::Vector3 scale(scale_property_->getFloat());
  arrow->setScale( scale );

  arrows_.push_back( arrow );

  if (!last_position_)    last_position_.reset(new Ogre::Vector3());
  if (!last_orientation_) last_orientation_.reset(new Ogre::Quaternion());
  *last_position_    = position;
  *last_orientation_ = orientation;
  context_->queueRender();
}

void Vector3Display::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
  clear();
}

void Vector3Display::update( float wall_dt, float ros_dt )
{
  size_t keep = keep_property_->getInt();
  if( keep > 0 )
  {
    while( arrows_.size() > keep )
    {
      delete arrows_.front();
      arrows_.pop_front();
    }
  }
}

void Vector3Display::reset()
{
  Display::reset();
  clear();
}

} // namespace hector_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hector_rviz_plugins::Vector3Display, rviz::Display )
