// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "detection_msgs/FrontPrediction.h"
#include "detection_msgs/HandlerPrediction.h"


namespace model_builder
{

class ModelBuilder
{

private:
    /**
     * @brief cur_processing_point_cloud - currently processing point cloud
     */
    ros::Publisher cur_processing_point_cloud;
    /**
     * @brief post_processed_point_cloud - post processed point cloud
     */
    ros::Publisher post_processed_point_cloud;
    ros::Publisher image_from_pcl_pub;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_to_process;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_output_cloud;

    /**
     * @brief is_waiting_for_front_prediction - Flag indicating that the node is waiting for image to be processed
     */
    bool is_waiting_for_front_prediction = false;
    bool is_waiting_for_handler_prediction = false;
    bool is_waiting_for_joint_prediction = false;

    /**
     * @brief setWaitForPredictionsFlags - sets all wait predictions flags to true.
     */
    void setWaitForPredictionsFlags(bool state);

    /**
     * @brief isAllPredictionsProcessed - checks if all predictions are ready
     * @return
     */
    bool isAllPredictionsReady();

    /**
     * @brief publishProcessedPointCloud - publish processed point cloud
     */
    void publishProcessedPointCloud();

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


    /**
     * @brief handlerPredictionCallback - callback to handler detection subscriber
     * @param handler_detection - handler detection
     */
    void handlerPredictionCallback(const detection_msgs::HandlerPredictionConstPtr& handler_detection);

};

}
