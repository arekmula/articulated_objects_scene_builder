#include "../include/model_builder/model_builder.hpp"
#include "time.h"

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
        front_prediction.processPrediction();

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
                                     std::vector<sensor_msgs::Image> in_masks,
                                     pcl::PointCloud<pcl::PointXYZRGB> in_cloud)
    {
        boxes = in_boxes;
        class_ids = in_class_ids;
        class_names = in_class_names;
        scores = in_scores;
        masks = in_masks;
        cloud = in_cloud;

        std::cout << "Number of front predictions: " << boxes.size() << std::endl;
    }

    FrontPrediction::~FrontPrediction()
    {

    }

    pcl::PointXYZRGB FrontPrediction::findRealCoordinatesFromImageCoordinates(int x, int y)
    {
        std::vector<int> dx = { 0, 1, 0, -1 };
        std::vector<int> dy = { 1, 0, -1, 0 };

        pcl::PointXYZRGB point = cloud(x, y);

        // If point not existing in point cloud, find nearest one
        int loop_count = 1;
        while (isnan(point.x))
        {
            std::transform(dx.begin(), dx.end(), dx.begin(), std::bind1st(std::multiplies<int>(), loop_count));
            std::transform(dy.begin(), dy.end(), dy.begin(), std::bind1st(std::multiplies<int>(), loop_count));
            for (int i=0; isnan(point.x) && i < 4; ++i)
            {
                point = cloud(x+ dx[i], y + dy[i]);
            }
            loop_count++;
        }

        return point;
    }

    FrontPrediction::prediction_color FrontPrediction::getPredictionColor(uint8_t class_id)
    {

        long int seed = ros::Time::now().toNSec();
        srand(seed);
        prediction_color colors={0, 0, 0};
        if (class_id == ROT_FRONT)
        {
            colors.r = int(rand() % 256);
            colors.g = int(rand() % 52);
            colors.b = int(rand() % 52);
            return colors;
        }
        else if (class_id == TRANS_FRONT)
        {
            colors.r = int(rand() % 52);
            colors.g = int(rand() % 256);
            colors.b = int(rand() % 52);
            return colors;
        }

        return colors;
    }

    void FrontPrediction::processPrediction()
    {

        uint8_t prediction_number = 0;
        for(std::vector<sensor_msgs::RegionOfInterest>::iterator it = boxes.begin(); it != boxes.end(); ++it)
        {
            int box_x_offset = it->x_offset;
            int box_y_offset = it->y_offset;
            int box_width = it->width;
            int box_height = it->height;
            uint8_t class_id = class_ids[prediction_number];
            prediction_color color = getPredictionColor(class_id);

            // Bottom left corner of bounding box
            pcl::PointXYZRGB bottom_left = findRealCoordinatesFromImageCoordinates(box_x_offset,
                                                                                   box_y_offset);
            // std::cout << bottom_left.x << " " << bottom_left.y << " " << bottom_left.z << std::endl;

            // Top left -> Not used atm
            /*
            pcl::PointXYZRGB top_left = findRealCoordinatesFromImageCoordinates(box_x_offset,
                                                                                box_y_offset + box_height);*/
            // std::cout << top_left.x << " " << top_left.y << " " << top_left.z << std::endl;

            // Bottom right -> Not used atm
            /*
            pcl::PointXYZRGB bottom_right = findRealCoordinatesFromImageCoordinates(box_x_offset + box_width,
                                                                                    box_y_offset);*/
            // std::cout << bottom_right.x << " " << bottom_right.y << " " << bottom_right.z << std::endl;

            pcl::PointXYZRGB top_right = findRealCoordinatesFromImageCoordinates(box_x_offset + box_width,
                                                                                 box_y_offset + box_height);
            //std::cout << top_right.x << " " << top_right.y << " " << top_right.z << std::endl;

            for (auto &point: cloud.points)
            {
                if (point.x > bottom_left.x && point.y > bottom_left.y
                        && point.x < top_right.x && point.y < top_right.y)
                {
                    if (class_id == ROT_FRONT)
                    {
                        point.r = color.r;
                        point.g = color.g;
                        point.b = color.b;
                    }
                    else if (class_id == TRANS_FRONT)
                    {
                        point.r = color.r;
                        point.g = color.g;
                        point.b = color.b;
                    }
                }
            }

            prediction_number++;
        }
    }
}
