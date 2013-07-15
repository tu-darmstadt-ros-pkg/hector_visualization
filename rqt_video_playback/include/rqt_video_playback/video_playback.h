//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef RQT_VIDEO_PLAYBACK_VIDEO_PLAYBACK_H
#define RQT_VIDEO_PLAYBACK_VIDEO_PLAYBACK_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_video_playback.h>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <Phonon/MediaObject>
#include <Phonon/MediaNode>

namespace rqt_video_playback {

class VideoPlayback
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  VideoPlayback();
  virtual ~VideoPlayback();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:
  void openClicked();
  void playClicked();
  void previousClicked();
  void nextClicked();
  void sliderValueChanged(int);

  void sync();

  void tick(qint64);
  void stateChanged(Phonon::State newstate, Phonon::State oldstate);
  void totalTimeChanged(qint64);
  void currentSourceChanged(Phonon::MediaSource);

signals:
  void clockUpdated();

protected:
  virtual void callbackClock(const rosgraph_msgs::Clock::ConstPtr& msg);
  Phonon::MediaObject *mediaObject() const { return ui_.videoPlayer->mediaObject(); }

private:
  ros::Duration toRos(qint64 time);
  qint64 fromRos(const ros::Duration& duration);

  Ui_VideoPlayback ui_;

  QWidget* widget_;

  ros::Subscriber subscriber_;

  ros::Time clock_;
  ros::Time start_;

private:
  qint64 step_time_;
  qint64 time_tolerance_;
};

} // namespace rqt_video_playback

#endif // RQT_VIDEO_PLAYBACK_VIDEO_PLAYBACK_H
