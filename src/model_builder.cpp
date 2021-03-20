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

    void FrontPrediction::processPrediction()
    {
        for(std::vector<sensor_msgs::RegionOfInterest>::iterator it = boxes.begin(); it != boxes.end(); ++it)
        {

            // Get bottom left vertices from ROI
            int x_bottom_left_vertice = it->x_offset;
            int y_bottom_left_vertice = it->y_offset;

            // Get top right vertices from ROI
            int x_top_right_vertice = it->x_offset + it->width;
            int y_top_right_vertice = it->y_offset + it->height;

            // Create list of x vertices
            pcl::Vertices x_box_vertices;
            x_box_vertices.vertices.push_back(x_bottom_left_vertice);
            x_box_vertices.vertices.push_back(x_top_right_vertice);

            // Create list of y vertices
            pcl::Vertices y_box_vertices;
            y_box_vertices.vertices.push_back(y_bottom_left_vertice);
            y_box_vertices.vertices.push_back(y_top_right_vertice);

            std::cout << "Bottom left: " << x_bottom_left_vertice << ", " << y_bottom_left_vertice << std::endl;
            std::cout << "Top right: " << x_top_right_vertice<< ", " << y_top_right_vertice << std::endl;

            std::vector<pcl::Vertices> box_vertices{x_box_vertices, y_box_vertices};

            pcl::CropHull<pcl::PointXYZRGB> cropFrontHull;
            // Set x,y vertices as hull indices
            cropFrontHull.setHullIndices(box_vertices);
            // Set cloud for CropHull class
            cropFrontHull.setHullCloud(cloud.makeShared());
            std::cout << cropFrontHull.getHullCloud()->size() << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
            cropFrontHull.filter(out_pointcloud);
            std::cout << out_pointcloud.size() << std::endl;
        }

    }
}
