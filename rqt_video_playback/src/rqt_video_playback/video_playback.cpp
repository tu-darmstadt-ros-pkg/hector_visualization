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

#include <rqt_video_playback/video_playback.h>
#include <pluginlib/class_list_macros.h>

#include <QFileDialog>

namespace rqt_video_playback {

VideoPlayback::VideoPlayback()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , step_time_(100)
  , time_tolerance_(100)
{
  setObjectName("VideoPlayback");
}

VideoPlayback::~VideoPlayback()
{
}

void VideoPlayback::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // connect UI elements
  connect(ui_.open, SIGNAL(clicked()), this, SLOT(openClicked()));
  connect(ui_.play, SIGNAL(clicked()), this, SLOT(playClicked()));
  connect(ui_.previous, SIGNAL(clicked()), this, SLOT(previousClicked()));
  connect(ui_.next, SIGNAL(clicked()), this, SLOT(nextClicked()));
  connect(ui_.slider, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
  connect(ui_.sync, SIGNAL(clicked()), this, SLOT(sync()));

  // configure video player
  mediaObject()->setTickInterval(100);
  connect(mediaObject(), SIGNAL(stateChanged(Phonon::State, Phonon::State)), this, SLOT(stateChanged(Phonon::State, Phonon::State)));
  connect(mediaObject(), SIGNAL(currentSourceChanged(Phonon::MediaSource)), this, SLOT(currentSourceChanged(Phonon::MediaSource)));
  connect(mediaObject(), SIGNAL(totalTimeChanged(qint64)), this, SLOT(totalTimeChanged(qint64)));

  // connect clock update
  connect(this, SIGNAL(clockUpdated()), this, SLOT(sync()));
  subscriber_ = getNodeHandle().subscribe<rosgraph_msgs::Clock>("/clock", 1, boost::bind(&VideoPlayback::callbackClock, this, _1));
}

bool VideoPlayback::eventFilter(QObject* watched, QEvent* event)
{
//  if (watched == ui_.image_frame && event->type() == QEvent::Paint)
//  {
//    QPainter painter(ui_.image_frame);
//    if (!qimage_.isNull())
//    {
//      // TODO: check if full draw is really necessary
//      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
//      //painter.drawImage(paint_event->rect(), qimage_);
//      qimage_mutex_.lock();
//      painter.drawImage(ui_.image_frame->contentsRect(), qimage_);
//      qimage_mutex_.unlock();
//    } else {
//      // default image with gradient
//      QLinearGradient gradient(0, 0, ui_.image_frame->frameRect().width(), ui_.image_frame->frameRect().height());
//      gradient.setColorAt(0, Qt::white);
//      gradient.setColorAt(1, Qt::black);
//      painter.setBrush(gradient);
//      painter.drawRect(0, 0, ui_.image_frame->frameRect().width() + 1, ui_.image_frame->frameRect().height() + 1);
//    }
//    return false;
//  }

  return QObject::eventFilter(watched, event);
}

void VideoPlayback::shutdownPlugin()
{
  subscriber_.shutdown();
  ui_.videoPlayer->stop();
  ui_.videoPlayer->close();
}

void VideoPlayback::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
//  QString topic = ui_.topics_combo_box->currentText();
//  //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
//  instance_settings.setValue("topic", topic);
//  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
//  instance_settings.setValue("dynamic_range", ui_.dynamic_range_check_box->isChecked());
//  instance_settings.setValue("max_range", ui_.max_range_double_spin_box->value());
}

void VideoPlayback::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
//  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
//  ui_.zoom_1_push_button->setChecked(zoom1_checked);

//  bool dynamic_range_checked = instance_settings.value("dynamic_range", false).toBool();
//  ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);

//  double max_range = instance_settings.value("max_range", ui_.max_range_double_spin_box->value()).toDouble();
//  ui_.max_range_double_spin_box->setValue(max_range);

//  QString topic = instance_settings.value("topic", "").toString();
//  //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
//  selectTopic(topic);
}

void VideoPlayback::openClicked()
{
  QString filter;

  QString fileName = QFileDialog::getOpenFileName(widget_, tr("Open Movie"),
          QDir::homePath(), filter);

  if (fileName.isEmpty()) return;

  ui_.videoPlayer->load(fileName);
  ui_.sync->setChecked(false);
  sync();
}

void VideoPlayback::playClicked()
{
  switch(mediaObject()->state()) {
    case Phonon::StoppedState:
      ui_.videoPlayer->play();
      break;
    case Phonon::PausedState:
      ui_.videoPlayer->play();
      break;
    case Phonon::PlayingState:
      ui_.videoPlayer->pause();
      break;
    default:
      qDebug() << "Unhandled Phonon state: " << mediaObject()->state();
      break;
  }
}

//void VideoPlayer::movieStateChanged(QMovie::MovieState state)
//{
//    switch(state) {
//    case QMovie::NotRunning:
//    case QMovie::Paused:
//        playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
//        break;
//    case QMovie::Running:
//        playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
//        break;
//    }
//}

//void VideoPlayer::frameChanged(int frame)
//{
//    if (!presentImage(movie.currentImage())) {
//        movie.stop();
//        playButton->setEnabled(false);
//        positionSlider->setMaximum(0);
//    } else {
//        positionSlider->setValue(frame);
//    }
//}

void VideoPlayback::previousClicked()
{
  qint64 new_time = ui_.videoPlayer->currentTime() - step_time_;
  if (new_time > 0)
    ui_.videoPlayer->seek(new_time);
}

void VideoPlayback::nextClicked()
{
  qint64 new_time = ui_.videoPlayer->currentTime() + step_time_;
  if (new_time < ui_.videoPlayer->totalTime())
    ui_.videoPlayer->seek(new_time);
}

void VideoPlayback::sliderValueChanged(int value)
{
  if (ui_.videoPlayer->currentTime() != value)
    ui_.videoPlayer->seek(value);
}

ros::Duration VideoPlayback::toRos(qint64 time)
{
  uint32_t sec, nsec;
  sec = time / 1000ll;
  nsec = (time - sec * 1000ll) * 1000000;
  return ros::Duration(sec, nsec);
}

qint64 VideoPlayback::fromRos(const ros::Duration& duration)
{
  return (qint64(duration.sec) * 1000ll + qint64(duration.nsec) / 1000000ll);
}

void VideoPlayback::sync()
{
  tick(ui_.videoPlayer->mediaObject()->currentTime());
}

void VideoPlayback::tick(qint64 tick)
{
  ui_.slider->setSliderPosition(tick);

  qint64 currentVideo = tick;
  qint64 totalTime = ui_.videoPlayer->totalTime();

  // is a video source loaded?
  if (ui_.videoPlayer->mediaObject()->currentSource().type() == Phonon::MediaSource::Empty) {
    ui_.controls->setEnabled(false);
    return;
  }

  // adjust start time if Sync is not checked
  if (!ui_.sync->isChecked()) {
    ui_.controls->setEnabled(true);

    if (!clock_.isZero()) start_ = clock_ - toRos(currentVideo);
    return;
  }

  // sync video with ROS clock time
  ui_.controls->setEnabled(false);

  // wait until clock is available
  if (clock_.isZero()) {
    ui_.videoPlayer->pause();
    return;
  }

  qint64 currentRos = fromRos(clock_ - start_);

  std::cout << "video: " << currentVideo << "/" << totalTime << ", ROS: " << currentRos << " => diff: " << (currentVideo - currentRos) << std::endl;

  if (currentVideo < currentRos - time_tolerance_) {
    if (currentRos < totalTime) {
      ui_.videoPlayer->seek(currentRos);
      ui_.videoPlayer->play();
    } else {
      ui_.videoPlayer->pause();
    }
  }

  if (currentVideo > currentRos + time_tolerance_) {
    if (currentRos >= 0) {
      ui_.videoPlayer->seek(currentRos);
      ui_.videoPlayer->play();
    } else {
      ui_.videoPlayer->pause();
    }
  }
}

void VideoPlayback::stateChanged(Phonon::State newstate, Phonon::State oldstate)
{
  switch(newstate) {
    case Phonon::PlayingState:
      ui_.play->setIcon(QIcon::fromTheme("media-playback-pause"));
      break;

    case Phonon::StoppedState:
      ui_.play->setIcon(QIcon::fromTheme("media-playback-start"));
      break;

    case Phonon::PausedState:
      ui_.play->setIcon(QIcon::fromTheme("media-playback-start"));
      break;
  }
}

void VideoPlayback::totalTimeChanged(qint64 total)
{
  ui_.slider->setMaximum(total);
}

void VideoPlayback::currentSourceChanged(Phonon::MediaSource)
{
  sync();
}

void VideoPlayback::callbackClock(const rosgraph_msgs::Clock::ConstPtr& clock)
{
  clock_ = clock->clock;
  clockUpdated();
}

} // namespace rqt_video_playback

PLUGINLIB_EXPORT_CLASS(rqt_video_playback::VideoPlayback, rqt_gui_cpp::Plugin)
