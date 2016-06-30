#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::Publisher detectedPub;

ros::Time start;
ros::Time detectTime;

float threshold = 5;
float init = 0.0;
bool detected = false;
std_msgs::Bool bool_msg;

float init_time = 1.0;
float detection_timeout = 3.0;

void co2_callback(const std_msgs::Float32::ConstPtr &msg){
    ROS_DEBUG("CO2 Callback");

    if((ros::Time::now() - start).toSec() < init_time){
      //ROS_INFO("Init");
      init = msg->data;
      return;
    }

    //ROS_INFO_THROTTLE(1, "time: %f", (ros::Time::now()- detectTime ).toSec());
    if(!detected && (msg->data - init) > threshold ){
      ROS_INFO("detect!");
      detectTime = ros::Time::now();
      bool_msg.data = true;
      detected = true;
    }else if(detected && (ros::Time::now()-detectTime).toSec() > detection_timeout){
      ROS_INFO("detect timeout");
      detected = false;
      bool_msg.data = false;
      init = msg->data;
    }

    if((init - msg->data) > 10){
      ROS_INFO("Resetting init");
      init = msg->data;
    }

    detectedPub.publish(bool_msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_co2_processing");


    ros::NodeHandle nh_;


    ros::Subscriber co2Sub = nh_.subscribe("input", 100, &co2_callback);

    detectedPub = nh_.advertise<std_msgs::Bool>("detected", 100);

    bool_msg.data = false;


    start = ros::Time::now();
    detectTime = ros::Time::now();

    ros::spin();
    exit(0);
}
