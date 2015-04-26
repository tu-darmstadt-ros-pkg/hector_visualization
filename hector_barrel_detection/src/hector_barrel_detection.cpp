#include <ros/ros.h>
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
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>

#include <message_filters/sync_policies/approximate_time.h>
namespace barrel_detection{
    BarrelDetection::BarrelDetection()
    {   ROS_INFO ("started!!!!!!!!!!!!!!");
        ros::NodeHandle pnh_("~");
        ros::NodeHandle nh_("");
        image_transport::ImageTransport it_(pnh_);

        pnh_.param("r_min", r_min, 0);
        pnh_.param("r_max", r_max, 10);
        pnh_.param("g_min", g_min, 0);
        pnh_.param("g_max", g_max, 10);
        pnh_.param("b_min", b_min, 10);
        pnh_.param("b_max", b_max, 255);
//        message_filters::Subscriber<sensor_msgs::Image> image_sub;
//        message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
//        image_sub.subscribe(nh_,"/openni/rgb/image_color", 1);
//        info_sub.subscribe(nh_, "/openni/camera_info", 1);

//        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximateTimeSyncPolicy;
//        message_filters::Synchronizer<ApproximateTimeSyncPolicy> sync_(ApproximateTimeSyncPolicy(5), image_sub, info_sub);



//        sync_.registerCallback(boost::bind(&BarrelDetection::imageCallback,this, _1, _2));
        //pcl_sub = nh_.subscribe("/openni/depth/points", 10, &BarrelDetection::PclCallback, this);
        image_sub = it_.subscribeCamera("/openni/rgb/image_color", 10, &BarrelDetection::imageCallback, this);
        cloud_filtered_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>       ("cloud_filtered_barrel", 0);
        pose_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>       ("pose_filtered_barrel", 0);
        barrel_marker_publisher_ = pnh_.advertise<visualization_msgs::MarkerArray>       ("marker_filtered_barrel", 0);
        percept_pub_   = nh_.advertise<hector_worldmodel_msgs::ImagePercept>       ("/worldmodel/image_percept", 0);

    }

    BarrelDetection::~BarrelDetection()
    {}

    void BarrelDetection::imageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info){
    //if(debug_){
     ROS_INFO("in callback of image!!!!!!!!!!!!!");
    //}


         //Read image with cvbridge
         cv_bridge::CvImageConstPtr cv_ptr;
         cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
         cv::Mat img_filtered(cv_ptr->image);

         cv::imshow("image",img_filtered);


         cv::Mat blueOnly;
         cv::inRange(img_filtered, cv::Scalar( b_min, g_min,r_min), cv::Scalar(b_max, g_max, r_max), blueOnly);

         cv::imshow("blau",blueOnly);
         cv::waitKey(1000);

//         if ((img_thres_.rows != static_cast<int>(img->height)) || (img_thres_.cols != static_cast<int>(img->width))){
//           img_thres_min_ = cv::Mat (img->height, img->width,CV_8UC1,1);
//           img_thres_max_ = cv::Mat (img->height, img->width,CV_8UC1,1);
//           img_thres_ = cv::Mat (img->height, img->width,CV_8UC1,1);
//        }


//       //Perform thresholding

//       //Define image thresholds for victim detection
//       int minThreshold = (int)std::max(std::min(((minTempVictim_ - mapping_.minTemp) *(256.0/( mapping_.maxTemp -  mapping_.minTemp))),255.0),0.0);
//       int maxThreshold = (int)std::max(std::min(((maxTempVictim_ - mapping_.minTemp) *(256.0/( mapping_.maxTemp -  mapping_.minTemp))),255.0),0.0);

//       cv::threshold(img_filtered,img_thres_min_,minThreshold,1,cv::THRESH_BINARY);
//       cv::threshold(img_filtered,img_thres_max_,maxThreshold,1,cv::THRESH_BINARY_INV);

//       //Element-wise multiplication to obtain an image with respect to both thresholds
//       IplImage img_thres_min_ipl = img_thres_min_;
//       IplImage img_thres_max_ipl = img_thres_max_;
//       IplImage img_thres_ipl = img_thres_;

//       cvMul(&img_thres_min_ipl, &img_thres_max_ipl, &img_thres_ipl, 255);

       //Perform blob detection
       cv::SimpleBlobDetector::Params params;
       params.filterByColor = true;
       params.blobColor = 255;
       params.minDistBetweenBlobs = 0.5;
       params.filterByArea = true;
       params.minArea = (blueOnly.rows * blueOnly.cols) /16;
       params.maxArea = blueOnly.rows * blueOnly.cols;
       params.filterByCircularity = false;
       params.filterByColor = false;
       params.filterByConvexity = false;
       params.filterByInertia = false;

       cv::SimpleBlobDetector blob_detector(params);
       std::vector<cv::KeyPoint> keypoints;
       keypoints.clear();

       blob_detector.detect(blueOnly,keypoints);
       for(unsigned int i=0; i<keypoints.size();i++)
       {
          std::cout << keypoints.at(i).pt.x << std::endl;
       }
       //Publish results
       hector_worldmodel_msgs::ImagePercept ip;

       ip.header= img->header;
       ip.info.class_id = "barrel";
       ip.info.class_support = 1;
       ip.camera_info =  *info;

       for(unsigned int i=0; i<keypoints.size();i++)
       {
           ip.x = keypoints.at(i).pt.x;
           ip.y = keypoints.at(i).pt.y;
           percept_pub_.publish(ip);
           ROS_INFO("Heat blob found at image coord: (%f, %f)", ip.x, ip.y);
       }


        }
    void BarrelDetection::PclCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
        ROS_INFO("started callback");
        pcl::PCLPointCloud2 pcl_pc2;
           pcl_conversions::toPCL(*pc_msg,pcl_pc2);
           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
           pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
           //do stuff with temp_cloud here



//              // Filtrar area
//              float xmin=-3.0,xmax=3.0;
//              float ymin=-3.0,ymax=3.0;
//              float zmin=-1.0,zmax=1.0;
//              ros::Time begin = ros::Time::now();
//              pcl::PassThrough<pcl::PointXYZ> pass;
//              pass.setInputCloud (cloud);
//              pass.setFilterFieldName ("x");
//              pass.setFilterLimits (xmin, xmax);
//              pass.filter (*cloud);
//              pass.setFilterFieldName ("y");
//              pass.setFilterLimits (ymin, ymax);
//              pass.filter (*cloud);
//              pass.setFilterFieldName ("z");
//              pass.setFilterLimits (zmin, zmax);
//              pass.filter (*cloud);
//              ROS_DEBUG_STREAM("Filtered by fields: " << cloud->points.size ());
//              ROS_DEBUG_STREAM("Fbf time:" << (ros::Time::now() - begin));

//              // downsample
//              begin = ros::Time::now();
//              pcl::VoxelGrid<pcl::PointXYZ> vg;
//              vg.setInputCloud (cloud);
//              vg.setLeafSize (0.05f, 0.05f, 0.05f);
//              vg.filter (*cloud);
//              ROS_DEBUG_STREAM("Downsampled: " << cloud->points.size ());
//              ROS_DEBUG_STREAM("VG time:" << (ros::Time::now() - begin));

//              // remove outliers
//              begin = ros::Time::now();
//              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//              sor.setInputCloud (cloud);
//              sor.setMeanK (50);
//              sor.setStddevMulThresh (0.1);
//              sor.filter (*cloud);
//              ROS_DEBUG_STREAM("Removed outliers: " << cloud->points.size ());
//              ROS_DEBUG_STREAM("SOR time:" << (ros::Time::now() - begin));

//              begin = ros::Time::now();
              // trobar cilindre(s)
              //Estimate point normals
              pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
              pcl::search::KdTree<pcl::PointXYZ>::Ptr  tree (new pcl::search::KdTree<pcl::PointXYZ> ());
              pcl::PointCloud<pcl::Normal>::Ptr        cloud_normals (new pcl::PointCloud<pcl::Normal> ());
              ne.setSearchMethod (tree);
              ne.setInputCloud (cloud);
              ne.setKSearch (5);
              ne.compute (*cloud_normals);
              //ROS_DEBUG_STREAM("Normals time:" << (ros::Time::now() - begin));

              // Create the segmentation object for cylinder segmentation and set all the parameters
            //  begin = ros::Time::now();
              pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
              pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
              pcl::PointIndices::Ptr      inliers_cylinder (new pcl::PointIndices);
              seg.setOptimizeCoefficients (true);
              seg.setModelType (pcl::SACMODEL_CYLINDER);
              seg.setMethodType (pcl::SAC_RANSAC);
              seg.setNormalDistanceWeight (0.1);
              seg.setMaxIterations (5000);
              seg.setDistanceThreshold (0.02);
              seg.setRadiusLimits (0.2,0.25);
              seg.setInputCloud (cloud);
              seg.setInputNormals (cloud_normals);
              seg.segment (*inliers_cylinder, *coefficients_cylinder);
              Eigen::Vector3f v = Eigen::Vector3f(0.0, 0.0, 1.0);
              seg.setAxis(v);
              ROS_DEBUG_STREAM("Cylinder coefficients: " << *coefficients_cylinder);
          //    ROS_DEBUG_STREAM("Seg time:" << (ros::Time::now() - begin));

           //   begin = ros::Time::now();
              pcl::ExtractIndices<pcl::PointXYZ> extract;
              extract.setInputCloud (cloud);
              extract.setIndices (inliers_cylinder);
              extract.setNegative (false);
              extract.filter (*cloud);
              ROS_INFO_STREAM("Extracted: " << cloud->points.size ());
              //ROS_DEBUG_STREAM("Extract time:" << (ros::Time::now() - begin));

              // Cylinder Cloud Publisher
              sensor_msgs::PointCloud2 cyl_msg;
              pcl::toROSMsg(*cloud,cyl_msg);
              cloud_filtered_publisher_.publish(cyl_msg);

              if( cloud->points.size()>0)
              { ROS_INFO("publish cylinder ");
                // POSE
                  geometry_msgs::PoseStamped PoseStamped_msg_;
                PoseStamped_msg_.header.stamp    = ros::Time::now();
                PoseStamped_msg_.header.frame_id = pc_msg->header.frame_id;
                PoseStamped_msg_.pose.position.x = coefficients_cylinder->values[0];
                PoseStamped_msg_.pose.position.y = coefficients_cylinder->values[1];
                PoseStamped_msg_.pose.position.z = coefficients_cylinder->values[2];
                PoseStamped_msg_.pose.orientation.x = PoseStamped_msg_.pose.orientation.y = PoseStamped_msg_.pose.orientation.z = 0;
                PoseStamped_msg_.pose.orientation.w = 1;
                pose_publisher_.publish(PoseStamped_msg_);
                // MARKERS ADD
                visualization_msgs::MarkerArray MarkerArray_msg_;
                MarkerArray_msg_.markers[0].action = MarkerArray_msg_.markers[1].action = visualization_msgs::Marker::ADD;
                 // CYLINDER AND TEXT
                MarkerArray_msg_.markers[0].header.frame_id = MarkerArray_msg_.markers[1].header.frame_id = pc_msg->header.frame_id;
                MarkerArray_msg_.markers[0].header.stamp = MarkerArray_msg_.markers[1].header.stamp = ros::Time::now();
                MarkerArray_msg_.markers[0].id = MarkerArray_msg_.markers[1].id = 0;
                MarkerArray_msg_.markers[0].pose = MarkerArray_msg_.markers[1].pose = PoseStamped_msg_.pose;
                MarkerArray_msg_.markers[0].lifetime = MarkerArray_msg_.markers[1].lifetime = ros::Duration();
                //red
                MarkerArray_msg_.markers[0].color.r = MarkerArray_msg_.markers[1].color.r = 1.0;
                MarkerArray_msg_.markers[0].color.g = MarkerArray_msg_.markers[1].color.g = 0.0;
                MarkerArray_msg_.markers[0].color.b = MarkerArray_msg_.markers[1].color.b = 0.0;
                // ONLY CYLINDER
                MarkerArray_msg_.markers[0].ns = "cylinder";
                MarkerArray_msg_.markers[0].type = visualization_msgs::Marker::CYLINDER;
                MarkerArray_msg_.markers[0].pose.position.z = -0.2;
                MarkerArray_msg_.markers[0].scale.x = 0.4;
                MarkerArray_msg_.markers[0].scale.y = 0.4;
                MarkerArray_msg_.markers[0].scale.z = 2;
                MarkerArray_msg_.markers[0].color.a = 0.4;
                // ONLY TEXT
                MarkerArray_msg_.markers[1].ns = "text";
                MarkerArray_msg_.markers[1].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                MarkerArray_msg_.markers[1].text = "blabla";
                MarkerArray_msg_.markers[1].pose.position.z = 1;
                MarkerArray_msg_.markers[1].scale.z = 0.3;
                MarkerArray_msg_.markers[1].color.a = 1.0;
  //           }else{
//                // MARKERS DELETE
//                MarkerArray_msg_.markers[0].action = MarkerArray_msg_.markers[1].action = visualization_msgs::Marker::DELETE;
//              }

             barrel_marker_publisher_.publish(MarkerArray_msg_);}
 //   }


}

}
