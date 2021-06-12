#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "pcl_conversions/pcl_conversions.h"

class Synchronizer{
private:

    /**
     * @brief should_process_received_cloud - value indicating if received cloud should be processed
     */
    bool should_process_received_cloud = true;
    /**
     * @brief image_from_pcl_pub - publisher for RGB image obtained from currently processed point cloud
     */
    ros::Publisher image_from_pcl_pub;

    /**
     * @brief point_cloud_pub - publisher for point cloud that is synchronized with the rgb image
     */
    ros::Publisher point_cloud_pub;

public:
    Synchronizer(ros::NodeHandle &node_handle)
    {
        // Create a ROS publisher for the input point cloud
        point_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_to_process", 1000);
        // Create a ROS publisher for the image generated from point cloud
        image_from_pcl_pub = node_handle.advertise<sensor_msgs::Image>("image_to_process", 1000);

        // When initializing node previous cloud is not post processed and we should allow to process received cloud
        should_process_received_cloud = true;

        std::cout << "Started synchronization node" << std::endl;
    }

    ~Synchronizer()
    {

    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_point_cloud)
    {
        if (should_process_received_cloud)
        {
            // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
            pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_to_process;
            pcl::fromROSMsg(*input_point_cloud, pcl_cloud_to_process);

            // Get RGB image from PointCloud and publish it so other nodes can generate prediction
            sensor_msgs::Image rgb_image;
            pcl::toROSMsg(pcl_cloud_to_process, rgb_image);

            // Publish synchronized point cloud and image from it
            point_cloud_pub.publish(*input_point_cloud);
            image_from_pcl_pub.publish(rgb_image);

            should_process_received_cloud = false;

        }
    }

    void processedPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& post_processed_point_cloud)
    {
        // Previous cloud has been processed. Set flag to true and allow to receive next point cloud to be processed.
        should_process_received_cloud = true;
    }
};



int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "synchronization_node");
    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    Synchronizer synchronizer(n);


    ros::Subscriber point_cloud_sub = n.subscribe("/hz_points",
                                                  1,
                                                  &Synchronizer::pointCloudCallback,
                                                  &synchronizer);
    ros::Subscriber post_processed_point_cloud_sub = n.subscribe("/processed_fronts_point_cloud",
                                                                 1,
                                                                 &Synchronizer::processedPointCloudCallback,
                                                                 &synchronizer);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

