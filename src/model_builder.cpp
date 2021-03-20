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
        if (ModelBuilder::isAllPredictionsReady())
        {
            // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
            pcl::fromROSMsg(*input_point_cloud, pcl_processed_cloud);

            // Get RGB image from PointCloud and publish it so other nodes can generate predictions
            sensor_msgs::Image rgb_image;
            pcl::toROSMsg(pcl_processed_cloud, rgb_image);
            image_from_pcl_pub.publish(rgb_image);

            // Publish current processed point cloud
            sensor_msgs::PointCloud2 output_point_cloud;
            output_point_cloud = *input_point_cloud;
            processed_point_cloud_pub.publish(output_point_cloud);

            // Set flags for waiting until all predictions on current point cloud will be processed
            ModelBuilder::setWaitForPredictionsFlags();
        }
        else
        {
            std::cout << "Waiting for all predictions on previous point cloud!" << std::endl;
        }
    }

    void ModelBuilder::frontPredictionCallback(const detection_msgs::FrontPredictionConstPtr &front_detection)
    {
        is_waiting_for_front_prediction = false;
        std::cout << "Received front prediction!" << std::endl;

        FrontPrediction front_prediction(front_detection->boxes,
                                         front_detection->class_ids,
                                         front_detection->class_names,
                                         front_detection->scores,
                                         front_detection->masks,
                                         pcl_processed_cloud);

    }


    void ModelBuilder::setWaitForPredictionsFlags()
    {
        is_waiting_for_front_prediction = true;
        // Currently not used
        //is_waiting_for_handler_prediction = true;
        //is_waiting_for_joint_prediction = true;
    }

    bool ModelBuilder::isAllPredictionsReady()
    {
        if (!is_waiting_for_front_prediction && !is_waiting_for_handler_prediction && !is_waiting_for_joint_prediction)
            return true;
        else
            return false;
    }


    FrontPrediction::FrontPrediction(std::vector<sensor_msgs::RegionOfInterest> in_boxes,
                                     std::vector<int32_t> in_class_ids,
                                     std::vector<std::string> in_class_names,
                                     std::vector<float_t> in_scores,
                                     std::vector<sensor_msgs::Image> in_masks
                                     , pcl::PointCloud<pcl::PointXYZRGB> in_cloud)
    {
        boxes = in_boxes;
        class_ids = in_class_ids;
        class_names = in_class_names;
        scores = in_scores;
        masks = in_masks;

        std::cout << "Number of front predictions: " << boxes.size() << std::endl;
    }

    FrontPrediction::~FrontPrediction()
    {

    }

    void FrontPrediction::processPrediction()
    {

    }
}
