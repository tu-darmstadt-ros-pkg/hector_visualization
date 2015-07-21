/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rqt_map_merger__MapMerger_H
#define rqt_map_merger__MapMerger_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_map_merger.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>


#include <opencv2/core/core.hpp>

#include <QImage>
#include <QList>
#include <QMutex>
#include <QString>
#include <QSize>
#include <QWidget>

#include <vector>


namespace rqt_map_merger {

class MapMerger
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  MapMerger();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid * map);
  nav_msgs::OccupancyGrid cvMatToOccupancyGrid(const cv::Mat * im);

  virtual void shutdownPlugin();
  bool eventFilter(QObject* watched, QEvent* event);

  void redraw_Label_1();
  void redraw_Label_2();
  void redraw_Label_merged();
  void timerCallback(const ros::TimerEvent& e);


public slots:
  void onSlider1Moved(int idx);
  void onSlider2Moved(int idx);
  void onMergePressed();
  void onSaveGeotiffPressed();
  void onStateUseTransformChanged(int checked);
  void onGetTransform();


protected:

  void map1Callback(nav_msgs::OccupancyGridConstPtr const & map);
  void map2Callback(nav_msgs::OccupancyGridConstPtr const & map);

protected slots:

  
protected:

  

  Ui::MapMergerWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;

  ros::Subscriber map_1_sub_;
  ros::Subscriber map_2_sub_;

  std::vector<QImage> qimages_1_;
  std::vector<cv::Mat> cv_maps_1_;
  std::vector<nav_msgs::OccupancyGrid> og_maps_1_;
  QImage qimage_1_;
  cv::Mat cv_map_1_;
  nav_msgs::OccupancyGrid og_map_1_;
  std::vector<QImage> qimages_2_;
  std::vector<cv::Mat> cv_maps_2_;
  std::vector<nav_msgs::OccupancyGrid> og_maps_2_;
  QImage qimage_2_;
  cv::Mat cv_map_2_;
  nav_msgs::OccupancyGrid og_map_2_;

  QImage qimage_merged_;
  QMutex qimage_mutex_;

  cv::Mat conversion_mat_;
  cv::Mat stored_transform;
  cv::Mat current_transform;

  ros::NodeHandle my_nh_;

  int current_idx_1_;
  int current_idx_2_;

  int stored_idx_1_;
  int stored_idx_2_;

  bool use_stored_transform_;
  int first_map_1_received;
  int first_map_2_received;

  ros::Timer map_save_timer_;

  ros::Publisher map_1_to_use_pub_;
  ros::Publisher map_2_to_use_pub_;
  ros::Publisher hector_command_pub_;

};

}

#endif // rqt_image_view__ImageView_H
