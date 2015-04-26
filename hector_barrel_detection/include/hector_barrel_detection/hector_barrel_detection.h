#ifndef HECTOR_BARREL_DETECTION_NODE_H
#define HECTOR_BARREL_DETECTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/camera_subscriber.h>
namespace barrel_detection{

    class BarrelDetection {

    public:
      BarrelDetection();
      virtual ~BarrelDetection();
    protected:
    void PclCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info);


    private:
      ros::Subscriber pcl_sub;
      image_transport::CameraSubscriber image_sub;
      ros::Publisher cloud_filtered_publisher_;
      ros::Publisher pose_publisher_;
      ros::Publisher barrel_marker_publisher_;
      ros::Publisher percept_pub_;

      int r_min;
      int r_max;
      int g_min;
      int g_max;
      int b_min;
      int b_max;


    };
}

namespace head_tracking_mode{
  const unsigned char NONE = 0;
  const unsigned char LEFT_HAND_TRACKING = 1;
  const unsigned char RIGHT_HAND_TRACKING = 2;
}
#endif // HECTOR_BARREL_DETECTION_NODE_H
