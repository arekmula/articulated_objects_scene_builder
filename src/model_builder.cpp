#include "../include/model_builder/model_builder.hpp"

namespace model_builder{

    ModelBuilder::ModelBuilder(ros::NodeHandle &node_handle)
    {
        // Create a ROS publisher for the output point cloud
        processed_point_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1000);

        // Create a ROS publisher for the image generated from point cloud
        image_from_pcl_pub = node_handle.advertise<sensor_msgs::Image>("image_to_process", 1000);
        std::cout << "Started Model builder" << std::endl;
    }

    ModelBuilder::~ModelBuilder()
    {

    }

    void ModelBuilder::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_point_cloud)
    {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*input_point_cloud, pcl_cloud);

        // Get RGB image from PointCloud and publish it so other nodes can generate predictions
        sensor_msgs::Image rgb_image;
        pcl::toROSMsg(pcl_cloud, rgb_image);
        image_from_pcl_pub.publish(rgb_image);

        // Publish current processed point cloud
        sensor_msgs::PointCloud2 output_point_cloud;
        output_point_cloud = *input_point_cloud;
        processed_point_cloud_pub.publish(output_point_cloud);
    }

    void ModelBuilder::frontPredictionCallback(const detection_msgs::FrontPredictionConstPtr &front_detection)
    {
        std::cout << "Received prediction!" << std::endl;
    }

}
