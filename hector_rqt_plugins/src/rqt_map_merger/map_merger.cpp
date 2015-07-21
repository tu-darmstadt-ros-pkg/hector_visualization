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

#include <rqt_map_merger/map_merger.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QMessageBox>
#include <QPainter>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Transform.h>
#include "hector_mapstitch/mapstitch.h"
#include "hector_mapstitch/utils.h"
#include <std_msgs/String.h>

namespace rqt_map_merger {

MapMerger::MapMerger()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("MapMerger");
}

void MapMerger::initPlugin(qt_gui_cpp::PluginContext& context)
{
    std::cout << "started map merging plugin " <<std::endl;
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ui_.map2_label->installEventFilter(this);
  ui_.map1_label->installEventFilter(this);

  map_1_sub_= my_nh_.subscribe("/original_map_robot_1", 1, &MapMerger::map1Callback,this);
  map_2_sub_= my_nh_.subscribe("/original_map_robot_2", 1, &MapMerger::map2Callback,this);

  map_1_to_use_pub_ = my_nh_.advertise<nav_msgs::OccupancyGrid>("/map_to_use_1",10,true);
  map_2_to_use_pub_ = my_nh_.advertise<nav_msgs::OccupancyGrid>("/map_to_use_2",10,true);

  hector_command_pub_ = my_nh_.advertise<std_msgs::String>("/syscommand", 10,true);

  //ros::ServiceServer servserv = my_nh_.advertiseService(stitched_map_topic + "_serive",stitchedMapService);


  //updateTopicList();
  //ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.map_hector1_horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onSlider1Moved(int)));
  connect(ui_.map_hector2_horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onSlider2Moved(int)));
  connect(ui_.merge_button, SIGNAL(pressed()), this, SLOT(onMergePressed()));
  connect(ui_.save_geotiff_button, SIGNAL(pressed()), this, SLOT(onSaveGeotiffPressed()));
  redraw_Label_1();
  redraw_Label_2();
   map_save_timer_ = my_nh_.createTimer(ros::Duration(10.0), &MapMerger::timerCallback, this, false );
  current_idx_1_ = 0;
  current_idx_2_ = 0;
}

 void MapMerger::timerCallback(const ros::TimerEvent& e){
  qimages_1_.push_back(qimage_1_);
  qimages_2_.push_back(qimage_2_);
  cv_maps_1_.push_back(cv_map_1_);
  cv_maps_2_.push_back(cv_map_2_);
  og_maps_1_.push_back(og_map_1_);
  og_maps_2_.push_back(og_map_2_);
  ui_.map_hector1_horizontalSlider->setRange(0,qimages_1_.size()-1);
  ui_.map_hector2_horizontalSlider->setRange(0,qimages_2_.size()-1);
 }

void MapMerger::onSlider1Moved(int idx){
  current_idx_1_ = idx;
  redraw_Label_1();
}

void MapMerger::onSlider2Moved(int idx){
  current_idx_2_ = idx;
  redraw_Label_2();
}

void MapMerger::onMergePressed(){
    redraw_Label_merged();
}

void MapMerger::onSaveGeotiffPressed(){
    std_msgs::String geo_string;
    geo_string.data = "savegeotiff";
    hector_command_pub_.publish(geo_string);
}

void MapMerger::redraw_Label_1(){
    if (qimages_1_.size()>0)
    {

      qimage_mutex_.lock();

      QImage image_scaled = qimages_1_[current_idx_1_].scaled(ui_.map1_label->width(), ui_.map1_label->height(), Qt::IgnoreAspectRatio );
       ui_.map1_label->setPixmap(QPixmap::fromImage(image_scaled));

      qimage_mutex_.unlock();


    } else {
      ROS_INFO("no map for robot1 known");
    }


}

void MapMerger::redraw_Label_2(){

    if (qimages_2_.size()>0)
    {
      qimage_mutex_.lock();


      QImage image_scaled = qimages_2_[current_idx_2_].scaled(ui_.map2_label->width(), ui_.map2_label->height(), Qt::IgnoreAspectRatio );
      ui_.map2_label->setPixmap(QPixmap::fromImage(image_scaled));
      qimage_mutex_.unlock();

    } else {
      ROS_INFO("no map for robot2 known");


    }


}

void MapMerger::redraw_Label_merged(){

     if (cv_maps_1_.size() > 0 && cv_maps_2_.size()>0 && og_maps_1_.size()>0 && og_maps_2_.size()>0){
    StitchedMap stitched_map(cv_maps_1_[current_idx_1_], cv_maps_2_[current_idx_2_]);

    if(stitched_map.isValid())
    {
        ui_.merge_button->setText("merging ...");
        cv::Mat stitched_cv_map = stitched_map.get_stitch();
        ui_.merge_button->setText("merge");
        cv::Mat gray;
        stitched_cv_map.convertTo(gray, CV_8UC1);
        cv::Mat temp;
        cv::cvtColor(gray, temp, CV_GRAY2RGB);


        QImage image(temp.data, temp.cols, temp.rows, QImage::Format_RGB888);
        QImage image_scaled = image.scaled(ui_.merged_map_label->width(), ui_.merged_map_label->height(), Qt::IgnoreAspectRatio );
        ui_.merged_map_label->setPixmap(QPixmap::fromImage(image_scaled));

        map_1_to_use_pub_.publish(og_maps_1_[current_idx_1_]);
        map_2_to_use_pub_.publish(og_maps_2_[current_idx_2_]);
    }

    }
}

bool MapMerger::eventFilter(QObject* watched, QEvent* event)
{/*bool successfull = true;

  if (watched == ui_.map2_label && event->type() == QEvent::Paint)
  {

    redraw_Label_2();
    //successfull = false;
  }

  if (watched == ui_.map1_label && event->type() == QEvent::Paint)
  {
    redraw_Label_1();
    //successfull = false;
  }

  if (successfull){
  return QObject::eventFilter(watched, event);}
  else{
      return false;
  }*/
    return QObject::eventFilter(watched, event);
}

void MapMerger::shutdownPlugin()
{
  
}

void MapMerger::map1Callback(nav_msgs::OccupancyGridConstPtr const & map)
{
    cv::Mat map1_cv = occupancyGridToCvMat(map.get());
    cv::Mat gray;
    map1_cv.convertTo(gray, CV_8UC1);
    cv::cvtColor(gray, conversion_mat_, CV_GRAY2RGB);


    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);

    qimage_mutex_.lock();
    qimage_1_ = image.copy();
    qimage_mutex_.unlock();

    cv_map_1_ = map1_cv;
    og_map_1_ = *map;




}

void MapMerger::map2Callback(nav_msgs::OccupancyGridConstPtr const & map)
{
    cv::Mat map2_cv = occupancyGridToCvMat(map.get());
    cv::Mat gray;
    map2_cv.convertTo(gray, CV_8UC1);
    cv::cvtColor(gray, conversion_mat_, CV_GRAY2RGB);
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);

    qimage_mutex_.lock();
    qimage_2_ = image.copy();
    qimage_mutex_.unlock();

    cv_map_2_ = map2_cv;
    og_map_2_ = *map;


}

nav_msgs::OccupancyGrid MapMerger::cvMatToOccupancyGrid(const cv::Mat * im)
{
    nav_msgs::OccupancyGrid map;

    map.info.height = im->rows;
    map.info.width = im->cols;

    map.data.resize(map.info.width * map.info.height);

    for(size_t i = 0; i < map.data.size(); ++i)
    {
        uint8_t map_val = im->data[i];
        if(map_val == 0) map.data[i] = 100;
        else if(map_val == 254) map.data[i] = 0;
        else map.data[i] = -1;
    }

    return map;
}


cv::Mat MapMerger::occupancyGridToCvMat(const nav_msgs::OccupancyGrid *map)
{
  uint8_t *data = (uint8_t*) map->data.data(),
           testpoint = data[0];
  bool mapHasPoints = false;

  cv::Mat im(map->info.height, map->info.width, CV_8UC1);

  // transform the map in the same way the map_saver component does
  for (size_t i=0; i<map->data.size(); i++)
  {
    if (data[i] == 0)        im.data[i] = 254;
    else if (data[i] == 100) im.data[i] = 0;
    else im.data[i] = 205;

    // just check if there is actually something in the map
    if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
    testpoint = data[i];
  }

  // sanity check
  if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }

  return im;
}

}
PLUGINLIB_EXPORT_CLASS(rqt_map_merger::MapMerger, rqt_gui_cpp::Plugin)

