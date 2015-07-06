#include <hector_barrel_detection/hector_barrel_detection.h>

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

    pcl_sub = nh_.subscribe("/openni/depth/points", 1, &BarrelDetection::PclCallback, this);
    //        image_sub = it_.subscribeCamera("/openni/rgb/image_color", 10, &BarrelDetection::imageCallback, this);
    cloud_filtered_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>       ("cloud_filtered_barrel", 0);
    pose_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>       ("pose_filtered_barrel", 0);
    barrel_marker_publisher_ = pnh_.advertise<visualization_msgs::MarkerArray>       ("marker_filtered_barrel", 0);
    imagePercept_pub_   = nh_.advertise<hector_worldmodel_msgs::ImagePercept>       ("/worldmodel/image_percept", 0);
    posePercept_pub_= nh_.advertise<hector_worldmodel_msgs::PosePercept>       ("/worldmodel/pose_percept", 0);
    pcl_debug_pub_=nh_.advertise<sensor_msgs::PointCloud2> ("barrel_pcl_debug", 0);

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
        imagePercept_pub_.publish(ip);
        ROS_INFO("Barrel blob found at image coord: (%f, %f)", ip.x, ip.y);
    }


}
void BarrelDetection::PclCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
    ROS_INFO("started callback");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    //transformation cloud to world
    //TODO:: change base_link to /map
    tf::StampedTransform transform_cloud_to_map;
    try{
        ros::Time time = pc_msg->header.stamp;
        listener_.waitForTransform("/base_link", pc_msg->header.frame_id,
                                   time, ros::Duration(3.0));
        listener_.lookupTransform("/base_link", pc_msg->header.frame_id,
                                  time, transform_cloud_to_map);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Lookup Transform failed: %s",ex.what());
        return;
    }

    tf::transformTFToEigen(transform_cloud_to_map, to_map_);

    // Transform to base_link
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloud_tmp, to_map_);
    cloud = cloud_tmp;
    cloud->header.frame_id= transform_cloud_to_map.frame_id_;



    // Filtrar area
    float zmin=0.2,zmax=1.1;
    pass_.setInputCloud (cloud);
    pass_.setFilterFieldName ("z");
    pass_.setFilterLimits (zmin, zmax);
    pass_.filter (*cloud);

    //              // downsample
    //              pcl::VoxelGrid<pcl::PointXYZ> vg;
    //              vg.setInputCloud (cloud);
    //              vg.setLeafSize (0.03f, 0.03f, 0.03f);
    //              vg.filter (*cloud);

    // Publish filtered cloud to ROS for debugging
    if (pcl_debug_pub_.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud, filtered_msg);
        filtered_msg.header.frame_id = cloud->header.frame_id;
        pcl_debug_pub_.publish(filtered_msg);
    }

    //              // remove outliers
    //              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //              sor.setInputCloud (cloud);
    //              sor.setMeanK (20);
    //              sor.setStddevMulThresh (0.1);
    //              sor.filter (*cloud);

    //              begin = ros::Time::now();

    // trobar cilindre(s)
    ROS_DEBUG("Normal Estimation");
    //Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ROS_DEBUG("building Tree");
    pcl::search::KdTree<pcl::PointXYZ>::Ptr  tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr        cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (50);
    ROS_DEBUG("estimate Normals");
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    ROS_DEBUG("Set Cylinder coefficients");
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      inliers_cylinder (new pcl::PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (50);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.1,0.4);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);
    ROS_INFO("search cylinders");
    Eigen::Vector3f v = Eigen::Vector3f(0.0, 0.0, 1.0);
    seg.setAxis(v);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    ROS_DEBUG_STREAM("Cylinder coefficients: " << *coefficients_cylinder);

    ROS_DEBUG("extract cylinder potins");
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud);
    ROS_INFO_STREAM("Extracted: " << cloud->points.size ());

    // Cylinder Cloud Publisher
    if (cloud_filtered_publisher_.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 cyl_msg;
        pcl::toROSMsg(*cloud, cyl_msg);
        cyl_msg.header.frame_id = cloud->header.frame_id;
        cloud_filtered_publisher_.publish(cyl_msg);
    }

    if( cloud->points.size()>0)
    { ROS_INFO("publish cylinder ");

        //Publish results
        hector_worldmodel_msgs::PosePercept pp;

        pp.header.frame_id= cloud->header.frame_id;
        pp.header.stamp= pc_msg->header.stamp;
        pp.info.class_id= "barrel";
        pp.info.class_support=1;
        pp.info.object_id="barrel";
        pp.info.object_support=1;
        pp.pose.pose.position.x= coefficients_cylinder->values[0];
        pp.pose.pose.position.y= coefficients_cylinder->values[1];
        pp.pose.pose.position.z= 0.6;
        pp.pose.pose.orientation.x= pp.pose.pose.orientation.y = pp.pose.pose.orientation.z= 0;
        pp.pose.pose.orientation.w= 1;

        posePercept_pub_.publish(pp);
        ROS_INFO("PosePercept published");

        // MARKERS ADD
        ROS_DEBUG("initialize markerArray");
        visualization_msgs::MarkerArray markerArray_msg_;
        markerArray_msg_.markers.resize(1);
        ROS_DEBUG("markerarry created");
        markerArray_msg_.markers[0].action = visualization_msgs::Marker::ADD;
        ROS_DEBUG("marker added");
        // CYLINDER AND TEXT
        markerArray_msg_.markers[0].header.frame_id = cloud->header.frame_id;
        markerArray_msg_.markers[0].header.stamp = pc_msg->header.stamp;
        markerArray_msg_.markers[0].id = 0;
        markerArray_msg_.markers[0].pose.position.x=  pp.pose.pose.position.x;
        markerArray_msg_.markers[0].pose.position.y =  pp.pose.pose.position.y;
        markerArray_msg_.markers[0].pose.position.z =  pp.pose.pose.position.z;
        markerArray_msg_.markers[0].pose.orientation.x=markerArray_msg_.markers[0].pose.orientation.y= markerArray_msg_.markers[0].pose.orientation.z= pp.pose.pose.orientation.x;
        markerArray_msg_.markers[0].pose.orientation.w=1;
        ROS_DEBUG("cylinder and text added");
        //red
        markerArray_msg_.markers[0].color.r = 1.0;
        markerArray_msg_.markers[0].color.g = 0.0;
        markerArray_msg_.markers[0].color.b = 0.0;
        ROS_DEBUG("color added");
        // ONLY CYLINDER
        markerArray_msg_.markers[0].ns = "cylinder";
        markerArray_msg_.markers[0].type = visualization_msgs::Marker::CYLINDER;
        markerArray_msg_.markers[0].pose.position.z = 0.6;
        markerArray_msg_.markers[0].scale.x = 0.6;
        markerArray_msg_.markers[0].scale.y = 0.6;
        markerArray_msg_.markers[0].scale.z = 1;
        markerArray_msg_.markers[0].color.a = 0.4;
        ROS_DEBUG("cylinder only added");
        ROS_DEBUG("makerArray complete");
        barrel_marker_publisher_.publish(markerArray_msg_);
        ROS_DEBUG("markerArray published");


}


}

}
