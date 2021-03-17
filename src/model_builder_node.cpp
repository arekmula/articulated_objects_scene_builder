#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher g_processed_point_cloud_pub;
ros::Publisher g_image_from_pcl_pub;

void pointCallback(const sensor_msgs::PointCloud2ConstPtr& input_point_cloud)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*input_point_cloud, pcl_cloud);

    // Get RGB image from PointCloud and publish it so other nodes can generate predictions
    sensor_msgs::Image rgb_image;
    pcl::toROSMsg(pcl_cloud, rgb_image);
    g_image_from_pcl_pub.publish(rgb_image);

    // Publish current processed point lcoud
    sensor_msgs::PointCloud2 output_point_cloud;
    output_point_cloud = *input_point_cloud;
    g_processed_point_cloud_pub.publish(output_point_cloud);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "model_builder");
    ros::NodeHandle n;

    // Create a ROS publisher for the output point cloud
    g_processed_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1000);
    // Create a ROS publisher for the image generated from point cloud
    g_image_from_pcl_pub = n.advertise<sensor_msgs::Image>("image_to_process", 1000);
    // ROS subscriber for the input point cloud
    ros::Subscriber point_cloud_sub = n.subscribe("/hz_points", 1, pointCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
