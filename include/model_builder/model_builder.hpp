// ROS specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Package specific includes
#include "../include/model_builder/prediction.hpp"
#include "detection_msgs/FrontPrediction.h"
#include "detection_msgs/HandlerPrediction.h"
#include "detection_msgs/JointPrediction.h"

namespace model_builder
{

float ARROW_SHAFT_DIAMETER = 0.02;
float ARROW_HEAD_DIAMETER = 0.02;
float ARROW_HEAD_LENGTH = 0.02;

class ModelBuilder
{

private:
    /**
     * @brief post_processed_point_cloud_pub - publisher for post processed point cloud
     */
    ros::Publisher post_processed_point_cloud_pub;
    /**
     * @brief last_processed_point_cloud_pub - last processed point cloud
     */
    ros::Publisher last_processed_point_cloud_pub;
    /**
     * @brief trans_fronts_normals_pub - publisher for marker array containing translational joints normals
     */
    ros::Publisher trans_fronts_normals_pub;
    /**
     * @brief rot_fronts_joints_pub - publisher for marker arrray containing rotational fronts joints
     */
    ros::Publisher rot_fronts_joints_pub;
    /**
     * @brief pcl_cloud_to_process - point cloud that needs to be processed
     */
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_to_process;
    /**
     * @brief pcl_output_cloud - post processed output cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_output_cloud;
    /**
     * @brief current_header - header of the currently processed point cloud
     */
    std_msgs::Header current_header;
    /**
     * @brief trans_fronts_points - vector of points containing translational point and its normal
     */
    std::vector<pcl::PointXYZRGBNormal> trans_fronts_points;
    /**
     * @brief trans_fronts_normal_marker_array - marker array containing markers of normals
     */
    visualization_msgs::MarkerArray::Ptr trans_fronts_normal_marker_array;
    /**
     * @brief joint_real_coordinates - list of coordinates of predicted joints
     */
    std::vector<rot_joint_coordinates> joint_real_coordinates;

    /**
     * @brief fronts_point_clouds - vector of separated front clouds. Each cloud is separate cloud of front
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fronts_point_clouds;

    /**
     * @brief rot_fronts_joints_marker_array - marker array containing markers of rot fronts joins
     */
    visualization_msgs::MarkerArray::Ptr rot_fronts_joints_marker_array;

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

    /**
     * @brief fillAndPublishNormalsMarkerArray - fills marker array of translational joints normals
     */
    void fillAndPublishTransNormalsMarkerArray();

    /**
     * @brief fillAndPublishJointsMarkerArray - fills marker array of rotational joints
     */
    void fillAndPublishRotJointsMarkerArray();

    const std::string trans_normals_namespace = "trans_normals";
    const std::string rot_joints_namespace = "rot_joints";

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

    /**
     * @brief jointPredictionCallback - callback to joint detection subscriber
     * @param joint_detection - joint detection
     */
    void jointPredictionCallback(const detection_msgs::JointPredictionConstPtr& joint_detection);

};

}
