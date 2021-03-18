// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "detection_msgs/FrontPrediction.h"

namespace model_builder
{

class ModelBuilder
{
public:

    /**
     * @brief ModelBuilder - Constructor
     */
    ModelBuilder(ros::NodeHandle& node_handle);

    /**
     * @brief ~ModelBuilder - Destructor
     */
    virtual ~ModelBuilder();

    /**
     * @brief pointCloudCallback - callback to point cloud subscriber
     * @param input_point_cloud - input PointCloud2 pointer
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_point_cloud);

    /**
     * @brief frontPredictionCallback - callback to front detection subscriber
     * @param front_detection - front prediction
     */
    void frontPredictionCallback(const detection_msgs::FrontPredictionConstPtr& front_detection);
private:
    ros::Publisher processed_point_cloud_pub;
    ros::Publisher image_from_pcl_pub;
};

}
