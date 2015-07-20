//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <hector_geotiff/map_writer_interface.h>
#include <hector_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

#include <pluginlib/class_loader.h>
#include <fstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/formatters.hpp>

#include <boost/tokenizer.hpp>

#include <opencv/highgui.h>
#include<opencv/cv.h>

#include <hector_worldmodel_msgs/GetObjectModel.h>

namespace hector_barrel_geotiff_plugin {

using namespace hector_geotiff;

class SemanticMapWriterPlugin : public MapWriterPluginInterface
{
public:
  SemanticMapWriterPlugin();
  virtual ~SemanticMapWriterPlugin();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface) = 0;

protected:

  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;
  ros::ServiceClient service_client_no_;


  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  std::string pkg_path;
   double barrel_threshold;
};

SemanticMapWriterPlugin::SemanticMapWriterPlugin()
  : initialized_(false)
{}

SemanticMapWriterPlugin::~SemanticMapWriterPlugin()
{}

void SemanticMapWriterPlugin::initialize(const std::string& name)
{
    ros::NodeHandle plugin_nh("~/" + name);
    std::string service_name_;

    plugin_nh.param("service_name", service_name_, std::string("worldmodel/get_object_model"));
    plugin_nh.param("draw_all_objects", draw_all_objects_, false);
    plugin_nh.param("class_id", class_id_, std::string());

    service_client_ = nh_.serviceClient<hector_worldmodel_msgs::GetObjectModel>(service_name_);

    initialized_ = true;
    this->name_ = name;
    ROS_INFO_NAMED(name_, "Successfully initialized hector_geotiff MapWriter plugin %s.", name_.c_str());
}

class BarrelMapWriter : public SemanticMapWriterPlugin
{
public:
  virtual ~BarrelMapWriter() {}

  void draw(MapWriterInterface *interface)
  {
    if (!initialized_) return;

    hector_worldmodel_msgs::GetObjectModel data;
    if (!service_client_.call(data)) {
      ROS_ERROR_NAMED(name_, "Cannot draw victims, service %s failed", service_client_.getService().c_str());
      return;
    }

    int counter =0;
    for(hector_worldmodel_msgs::ObjectModel::_objects_type::const_iterator it = data.response.model.objects.begin(); it != data.response.model.objects.end(); ++it) {
      const hector_worldmodel_msgs::Object& object = *it;
      if (!class_id_.empty() && object.info.class_id != class_id_) continue;
      Eigen::Vector2f coords;
      coords.x() = object.pose.pose.position.x;
      coords.y() = object.pose.pose.position.y;
      interface->drawObjectOfInterest(coords, boost::lexical_cast<std::string>(++counter), MapWriterInterface::Color(180,0,200));


    }
  }
};

//class BarrelMapWriter : public SemanticMapWriterPlugin
//{
//public:
//  virtual ~BarrelMapWriter() {}

//  void draw(MapWriterInterface *interface)
//  {
//    if (!initialized_) return;

//    nav_msgs::GetMap map_barrels;
//    nav_msgs::GetMap map_no_barrels;
//    if (!service_client_.call(map_barrels)) {
//      ROS_ERROR_NAMED(name_, "Cannot draw barrels, service %s failed", service_client_.getService().c_str());
//      return;
//    }


//    cv::Mat cv_map_small, cv_map;

//    cv_map_small= occupancyGridToCvMat(&map_barrels.response.map);

//    cv::Mat cv_map_small_bw;
//    cv::threshold( cv_map_small, cv_map_small_bw, 0.1, 255, 0);


//    cv::copyMakeBorder( cv_map_small_bw, cv_map, 50, 50, 50, 50, cv::BORDER_CONSTANT, cv::Scalar(255) );


//    cv::imshow("map_big", cv_map);

//    cv::imwrite(pkg_path+"/templates/map.png",cv_map);
//    cv::waitKey(2000);
//    cv::Mat barrel_template = cv::imread(pkg_path+"/templates/barrel_3.png");

//      cv::Mat gref, gtpl, tmpl_gray;
//      cv::cvtColor(barrel_template, tmpl_gray, CV_BGR2GRAY);


//      gref = getSobel(cv_map);

//      ROS_WARN("blablabla    ");
//      cv::Mat ref = gref;
//      gtpl = getSobel(tmpl_gray);
//      ROS_WARN("blublublu    ");
//      cv::Mat tpl = gtpl;
////        cv::Mat ref = cv_map;
////        cv::Mat tpl = barrel_template;

//        if (ref.empty() || tpl.empty()) {
//            ROS_WARN("Empty map layer for barrels");
//            return;
//        }
//        else{

//        cv::Mat res(ref.rows-tpl.rows+1, ref.cols-tpl.cols+1, CV_32FC1);
//        cv::matchTemplate(gref, gtpl, res, CV_TM_SQDIFF_NORMED);

//        cv::threshold(res, res, barrel_threshold, 1., CV_THRESH_TOZERO);

//        std::vector<cv::Point> barrel_locations;
//        while (true)
//        {
//            double minval, maxval, threshold = barrel_threshold;
//            cv::Point minloc, maxloc;
//            cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);

//             ROS_INFO("max %d, %d",maxloc.x,maxloc.y);
//            if (maxval >= threshold)
//            {
//                cv::rectangle(
//                    ref,
//                    maxloc,
//                    cv::Point(maxloc.x + tpl.cols, maxloc.y + tpl.rows),
//                    cv::Scalar(255,0,0), 2
//                );
//                barrel_locations.push_back(maxloc);
//                cv::floodFill(res, maxloc, cv::Scalar(0), 0, cv::Scalar(.1), cv::Scalar(1.));
//            }
//            else
//                break;
//        }
//        std::cout << barrel_locations <<std::endl;
//        //cv::namedWindow( "reference", CV_WINDOW_AUTOSIZE );

//        cv::imshow("result", ref);
//        cv::waitKey(2000);
//        int counter = 0;
//        double pixelsPerMapMeter = 1.0f / map_barrels.response.map.info.resolution;

//        Eigen::Vector2f origin = Eigen::Vector2f(map_barrels.response.map.info.origin.position.x, map_barrels.response.map.info.origin.position.y);
//        std::cout<< "origin    :" <<origin <<std::endl;
//        for (int i=0;i<barrel_locations.size();i++){
//            cv::Point barrel_point = barrel_locations.at(i);
//        Eigen::Vector2f coords;
//        coords.x() = (barrel_point.x+50)/pixelsPerMapMeter;
//        coords.y() = (barrel_point.y+50)/pixelsPerMapMeter;
//        interface->drawObjectOfInterest(coords, boost::lexical_cast<std::string>(++counter), MapWriterInterface::Color(0,255,0));

//        std::cout<< "pixels per map !!!!!!!!!!!!!!!!!!!!!!!:"<<pixelsPerMapMeter <<std::endl;
//        std::cout<< "coords !!!!!!!!!!!!!!!!!!!!!!!:"<<coords <<std::endl;
//        interface->drawObjectOfInterest(coords, boost::lexical_cast<std::string>(++counter), MapWriterInterface::Color(0,255,0));

//        }
//   }
//  }

//  cv::Mat getSobel(cv::Mat src_gray){

//      cv::Mat grad;

//        int scale = 1;
//        int delta = 0;
//        int ddepth = CV_16S;

//        int c;

//      cv::GaussianBlur( src_gray, src_gray, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

//      /// Convert it to gray

//      /// Create window


//      /// Generate grad_x and grad_y
//      cv::Mat grad_x, grad_y;
//      cv::Mat abs_grad_x, abs_grad_y;

//      /// Gradient X
//      //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
//      cv::Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
//      convertScaleAbs( grad_x, abs_grad_x );

//      /// Gradient Y
//      //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
//      Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
//      convertScaleAbs( grad_y, abs_grad_y );

//      /// Total Gradient (approximate)
//      cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
//      ROS_WARN("gradient ");
//    // cv::imshow( "sobel", grad );
//      return grad;
//  }

//  cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid *map)
//  {
//    uint8_t *data = (uint8_t*) map->data.data(),
//             testpoint = data[0];
//    bool mapHasPoints = false;

//    cv::Mat im(map->info.height, map->info.width, CV_8UC1);

//    // transform the map in the same way the map_saver component does
//    for (size_t i=0; i<map->data.size(); i++)
//    {
//      if (data[i] == 0)        im.data[i] = 254;
//      else if (data[i] == 100) im.data[i] = 0;
//      else im.data[i] = 205;

//      // just check if there is actually something in the map
//      if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
//      testpoint = data[i];
//    }

//    // sanity check
//    if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }

//    return im;
//  }
//};



} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_barrel_geotiff_plugin::BarrelMapWriter, hector_geotiff::MapWriterPluginInterface)

