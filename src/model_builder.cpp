// Package specific includes
#include "../include/model_builder/model_builder.hpp"
#include "../include/model_builder/prediction.hpp"

// ROS specific includes
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>


namespace model_builder{

    ModelBuilder::ModelBuilder(ros::NodeHandle &node_handle)
    {
        // Create a ROS publisher for the currently processed point cloud
        cur_processing_point_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("currently_processed_point_cloud", 1000);

        // Create a ROS publisher for the post processed point cloud
        post_processed_point_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1000);

        // Create a ROS publisher for the marker array of normals of the trans fronts
        trans_fronts_normals_pub = node_handle.advertise<visualization_msgs::MarkerArray>("trans_fronts_normals", 1000);

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
            std::cout << "\nNew point cloud to process!" << std::endl;

            // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
            pcl::fromROSMsg(*input_point_cloud, pcl_cloud_to_process);
            pcl_output_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl_output_cloud->header = pcl_cloud_to_process.header;

            // Save current input cloud header for later usage
            current_header = input_point_cloud->header;

            // Clear array of translational points and its marker array
            trans_fronts_points.clear();
            trans_fronts_normal_marker_array.reset(new visualization_msgs::MarkerArray);

            // Get RGB image from PointCloud and publish it so other nodes can generate predictions
            sensor_msgs::Image rgb_image;
            pcl::toROSMsg(pcl_cloud_to_process, rgb_image);
            image_from_pcl_pub.publish(rgb_image);

            // Publish current processed point cloud
            sensor_msgs::PointCloud2 currently_processed_point_cloud;
            currently_processed_point_cloud = *input_point_cloud;
            cur_processing_point_cloud_pub.publish(currently_processed_point_cloud);

            // Set flags for waiting until all predictions on current point cloud will be processed
            ModelBuilder::setWaitForPredictionsFlags(true);
        }
        else
        {
            std::cout << "Waiting for all predictions on previous point cloud!" << std::endl;
        }
    }

    void ModelBuilder::frontPredictionCallback(const detection_msgs::FrontPredictionConstPtr &front_detection)
    {
        std::cout << "Received front prediction!" << std::endl;

        FrontPrediction front_prediction(front_detection->boxes,
                                         front_detection->class_ids,
                                         front_detection->class_names,
                                         front_detection->scores,
                                         front_detection->masks,
                                         pcl_cloud_to_process);
        front_prediction.processPrediction(pcl_output_cloud, true, trans_fronts_points);
        is_waiting_for_front_prediction = false;

        if (ModelBuilder::isAllPredictionsReady())
        {
            fillAndPublishMarkerArray();
            publishProcessedPointCloud();
        }
    }

    void ModelBuilder::handlerPredictionCallback(const detection_msgs::HandlerPredictionConstPtr &handler_detection)
    {
        std::cout << "Received handler prediction!" << std::endl;

        HandlerPrediction handler_prediction(handler_detection->boxes,
                                             handler_detection->class_ids,
                                             handler_detection->class_names,
                                             handler_detection->scores,
                                             handler_detection->masks,
                                             pcl_cloud_to_process);
        handler_prediction.processPrediction(pcl_output_cloud, false, trans_fronts_points);
        is_waiting_for_handler_prediction = false;

        if (ModelBuilder::isAllPredictionsReady())
        {
            fillAndPublishMarkerArray();
            publishProcessedPointCloud();
        }
    }

    void ModelBuilder::jointPredictionCallback(const detection_msgs::JointPredictionConstPtr &joint_detection)
    {
        std::cout << "Received joint prediction!" << std::endl;

        JointPrediction joint_prediction(joint_detection->x1,
                                         joint_detection->y1,
                                         joint_detection->x2,
                                         joint_detection->y2,
                                         pcl_cloud_to_process);
        joint_prediction.processPrediction(pcl_output_cloud);
        is_waiting_for_joint_prediction = false;

        if (ModelBuilder::isAllPredictionsReady())
        {
            fillAndPublishMarkerArray();
            publishProcessedPointCloud();
        }
    }

    void ModelBuilder::setWaitForPredictionsFlags(bool state)
    {
        is_waiting_for_front_prediction = state;
        is_waiting_for_handler_prediction = state;
        is_waiting_for_joint_prediction = state;
    }

    bool ModelBuilder::isAllPredictionsReady()
    {
        if (!is_waiting_for_front_prediction && !is_waiting_for_handler_prediction && !is_waiting_for_joint_prediction)
            return true;
        else
            return false;
    }

    void ModelBuilder::publishProcessedPointCloud()
    {
        std::cout << "*****************Publishing processed point cloud*******************" << std::endl;
        sensor_msgs::PointCloud2 output_point_cloud;
        pcl::toROSMsg(*pcl_output_cloud, output_point_cloud);
        post_processed_point_cloud_pub.publish(output_point_cloud);
    }

    void ModelBuilder::fillAndPublishMarkerArray()
    {
        int current_marker_id = 0;
        for (auto normal = trans_fronts_points.begin(); normal!=trans_fronts_points.end(); ++normal)
        {
            geometry_msgs::Point p1;
            p1.x = normal->x;
            p1.y = normal->y;
            p1.z = normal->z;

            geometry_msgs::Point p2;
            p2.x = p1.x + normal->normal_x;
            p2.y = p1.y + normal->normal_y;
            p2.z = p1.z + normal->normal_z;

            visualization_msgs::Marker current_marker;
            current_marker.header = current_header;
            current_marker.type = visualization_msgs::Marker::ARROW;
            current_marker.action = visualization_msgs::Marker::ADD;
            current_marker.id = current_marker_id;
            current_marker.color.a = 1.0;
            current_marker.color.r = float((normal->r) / 255.0);
            current_marker.color.g = float((normal->g) / 255.0);
            current_marker.color.b = float((normal->b) / 255.0);

            current_marker.scale.x = ARROW_SHAFT_DIAMETER;
            current_marker.scale.y = ARROW_HEAD_DIAMETER;
            current_marker.scale.z = ARROW_HEAD_LENGTH;

            current_marker.points.push_back(p1);
            current_marker.points.push_back(p2);

            trans_fronts_normal_marker_array->markers.push_back(current_marker);

            current_marker_id++;
        }

        trans_fronts_normals_pub.publish(trans_fronts_normal_marker_array);
    }
}
