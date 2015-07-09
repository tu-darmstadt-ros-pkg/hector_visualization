#ifndef HECTOR_BARREL_DETECTION_NODE_H
#define HECTOR_BARREL_DETECTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <hector_barrel_detection/hector_barrel_detection.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/filters/voxel_grid.h>
 #include <pcl/filters/statistical_outlier_removal.h>
 #include <pcl/ModelCoefficients.h>
 #include <pcl/features/normal_3d.h>
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/segmentation/sac_segmentation.h>
 #include <pcl/filters/extract_indices.h>
 #include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv/highgui.h>
#include<opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <hector_worldmodel_msgs/ImagePercept.h>
#include <hector_worldmodel_msgs/PosePercept.h>
#include <hector_worldmodel_msgs/GetObjectModel.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <hector_nav_msgs/GetDistanceToObstacle.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

namespace barrel_detection{

    class BarrelDetection {

    public:
      BarrelDetection();
      virtual ~BarrelDetection();
    protected:
    void PclCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info);
    void findCylinder(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, float xKey, float yKey);


    private:
      typedef boost::shared_ptr<image_geometry::PinholeCameraModel> CameraModelPtr;

      ros::Subscriber pcl_sub;
      image_transport::CameraSubscriber image_sub;
      ros::Publisher cloud_filtered_publisher_;
      ros::Publisher pose_publisher_;
      ros::Publisher barrel_marker_publisher_;
      ros::Publisher imagePercept_pub_;
      ros::Publisher posePercept_pub_;
      ros::ServiceClient worldmodel_srv_client_;
      tf::TransformListener listener_;
      Eigen::Affine3d to_map_;
      pcl::PassThrough<pcl::PointXYZ> pass_;

      ros::Publisher debug_imagePoint_pub_;
      ros::Publisher pcl_debug_pub_;

      sensor_msgs::PointCloud2::ConstPtr current_pc_msg_;

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
