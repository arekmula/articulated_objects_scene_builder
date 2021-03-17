#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher g_test_point_pub;

void pointCallback(const sensor_msgs::PointCloud2ConstPtr& input_point_cloud)
{
    sensor_msgs::PointCloud2 output_point_cloud;
    output_point_cloud = *input_point_cloud;
    g_test_point_pub.publish(output_point_cloud);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "model_builder");
    ros::NodeHandle n;

    // Create a ROS publisher for the output point cloud
    g_test_point_pub = n.advertise<sensor_msgs::PointCloud2>("test_point_cloud", 1000);

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
