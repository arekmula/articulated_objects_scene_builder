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
    pcl::PointCloud<pcl::PointXYZRGB> pcl_processed_cloud;

    /**
     * @brief is_waiting_for_front_prediction - Flag indicating that the node is waiting for image to be processed
     */
    bool is_waiting_for_front_prediction = false;
    bool is_waiting_for_handler_prediction = false;
    bool is_waiting_for_joint_prediction = false;

    /**
     * @brief setWaitForPredictionsFlags - sets all wait predictions flags to true.
     */
    void setWaitForPredictionsFlags();

    /**
     * @brief isAllPredictionsProcessed - checks if all predictions are ready
     * @return
     */
    bool isAllPredictionsReady();

};

class FrontPrediction
{
public:
    /**
     * @brief FrontPrediction - Constructor
     */
    FrontPrediction(std::vector<sensor_msgs::RegionOfInterest> in_boxes,
                    std::vector<int32_t> in_class_ids,
                    std::vector<std::string> in_class_names,
                    std::vector<float_t> in_scores,
                    std::vector<sensor_msgs::Image> in_masks,
                    pcl::PointCloud<pcl::PointXYZRGB> in_cloud);

    /**
     * @brief ~FrontPrediction - Destructor
     */
    virtual ~FrontPrediction();

    /**
     * @brief processPrediction - processing prediction on point cloud
     */
    void processPrediction();

private:
    std::vector<sensor_msgs::RegionOfInterest> boxes;
    std::vector<int32_t> class_ids;
    std::vector<std::string> class_names;
    std::vector<float_t> scores;
    std::vector<sensor_msgs::Image> masks;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

};

}
