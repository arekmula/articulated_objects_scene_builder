#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "../include/model_builder/model_builder.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "model_builder");
    ros::NodeHandle n;

    model_builder::ModelBuilder modelBuilder(n);

    // ROS subscriber for the input point cloud
    ros::Subscriber point_cloud_sub = n.subscribe("/hz_points", 1, &model_builder::ModelBuilder::pointCloudCallback, &modelBuilder);

    // ROS subscriber for the front prediction
    // ros::Subscriber front_prediction_sub = n.subscribe("front_prediction_topic", 1, frontPredictionTopic);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
