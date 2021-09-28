#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::Publisher detectedPub;

float filtered_value;
float threshold;
float timeout_duration_sec;
ros::Time last_ramp;


void co2_callback(const std_msgs::Float32::ConstPtr &msg){
    if(filtered_value == 0.f) filtered_value = msg->data;
    filtered_value = 0.9 * filtered_value + 0.1 * msg->data;
    float delta = msg->data - filtered_value;
    ros::Time current_time = ros::Time::now();
    if(delta > threshold) {
      last_ramp = current_time;
    }

    std_msgs::Bool bool_msg;
    bool_msg.data = (current_time - last_ramp).toSec() < timeout_duration_sec;
    detectedPub.publish(bool_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_co2_processing_node");
    ros::NodeHandle nh_;
    filtered_value = 0.f;
    threshold = 50.f;
    timeout_duration_sec = 2.0;
    ros::Subscriber co2Sub = nh_.subscribe("/co2", 1, &co2_callback);
    detectedPub = nh_.advertise<std_msgs::Bool>("/co2detected", 1);
    ros::spin();
    exit(0);
}
