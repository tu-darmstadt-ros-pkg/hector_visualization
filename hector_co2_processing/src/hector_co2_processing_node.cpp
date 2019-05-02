#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::Publisher co2Pub;
ros::Publisher detectedPub;

float avg;

float threshold;

float scale;

void co2_callback(const std_msgs::Float32::ConstPtr &msg){
    ROS_DEBUG("CO2 Callback");
    ROS_DEBUG("avg_old: %f", avg);
    avg = 0.9 * avg + 0.1 * msg->data;

    ROS_DEBUG("avg: %f", avg);
    double value = fabs(msg->data - avg);

    ROS_DEBUG("Value: %f", value);
    std_msgs::Float32 value_msg;
    value_msg.data = scale * value;
    co2Pub.publish(value_msg);

    std_msgs::Bool bool_msg;
    bool_msg.data = value > threshold;
    detectedPub.publish(bool_msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_co2_processing_node");

    ros::NodeHandle nh_;

    avg = 0;
    threshold = 50;
    scale = 75.0;

    ros::Subscriber co2Sub = nh_.subscribe("/co2", 100, &co2_callback);

    co2Pub = nh_.advertise<std_msgs::Float32>("/processed_co2", 100);
    detectedPub = nh_.advertise<std_msgs::Bool>("/co2detected", 100);

    ros::spin();
    exit(0);
}
