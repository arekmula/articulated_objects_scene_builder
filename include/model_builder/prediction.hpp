// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include "detection_msgs/FrontPrediction.h"
#include "detection_msgs/HandlerPrediction.h"

namespace model_builder {

struct joint_prediction_vertices{
    int32_t x1;
    int32_t y1;
    int32_t x2;
    int32_t y2;
};

class Prediction
{

protected:
    std::vector<sensor_msgs::RegionOfInterest> boxes;
    std::vector<int32_t> class_ids;
    std::vector<std::string> class_names;
    std::vector<float_t> scores;
    std::vector<sensor_msgs::Image> masks;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    /**
     * @brief findRealCoordinatesFromImageCoordinates - find real coordinates in cloud based on image coordinates
     * @param x - x image coordinate
     * @param y - y image coordinate
     * @return point containing real coordinates
     */
    pcl::PointXYZRGB findRealCoordinatesFromImageCoordinates(int x, int y);

    /**
     * @brief getBoundingBoxInliers - Gets bounding box inliers indices from point cloud which is stored in 1D
     *  organized array
     * @param boundingbox_inliers_indices - point indices with bounding box inliers
     * @param box_y_offset
     * @param box_height
     * @param box_x_offset
     * @param box_width
     */
    void getBoundingBoxInliersIndices(pcl::PointIndices::Ptr boundingbox_inliers_indices,
                                      int box_y_offset,
                                      int box_height,
                                      int box_x_offset,
                                      int box_width);

    /**
     * @brief extractCloudFromIndices - extract cloud based on  inliers indices
     * @param boundingbox_inliers_indices
     * @param extracted_cloud
     */
    void extractCloudFromIndices(pcl::PointIndices::Ptr indices,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud);
    /**
     * @brief extractCloudFromIndices - extract cloud based on  inliers indices
     * @param input cloud - if other than cloud stored in class
     * @param boundingbox_inliers_indices
     * @param extracted_cloud
     */
    void extractCloudFromIndices(pcl::PointIndices::Ptr indices,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud);

    /**
     * @brief findPlane - finds plane in input cloud
     * @param input_cloud
     * @param should_optimize_coeffficients
     * @param model_type
     * @param method_type
     * @param distance_threshold
     * @param plane_inliers - output plane inliers
     * @param plane_coefficients - output plane coefficients
     */
    void findPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, bool should_optimize_coeffficients,
                   pcl::SacModel model_type, const int method_type, float distance_threshold,
                   pcl::PointIndices::Ptr plane_inliers, pcl::ModelCoefficients::Ptr plane_coefficients);

    struct prediction_color{
        int r;
        int g;
        int b;
    };

public:
    /**
     * @brief Prediction - Constructor
     */
    Prediction(std::vector<sensor_msgs::RegionOfInterest> in_boxes,
                    std::vector<int32_t> in_class_ids,
                    std::vector<std::string> in_class_names,
                    std::vector<float_t> in_scores,
                    std::vector<sensor_msgs::Image> in_masks,
                    pcl::PointCloud<pcl::PointXYZRGB> in_cloud);

    /**
     * @brief ~Prediction - Destructor
     */
    virtual ~Prediction();

    /**
     * @brief processPrediction - processing prediction on point cloud
     */
    void processPrediction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud);

    virtual prediction_color getPredictionColor(uint8_t class_id);

};

class HandlerPrediction
        : public Prediction{

private:
    enum class_ids_names{
        NONE=0,
        HANDLER=1,
    };

public:
    HandlerPrediction(std::vector<sensor_msgs::RegionOfInterest> in_boxes,
                       std::vector<int32_t> in_class_ids,
                       std::vector<std::string> in_class_names,
                       std::vector<float_t> in_scores,
                       std::vector<sensor_msgs::Image> in_masks,
                       pcl::PointCloud<pcl::PointXYZRGB> in_cloud):Prediction(in_boxes,
                                                                              in_class_ids,
                                                                              in_class_names,
                                                                              in_scores,
                                                                              in_masks,
                                                                              in_cloud)
    {}

    ~HandlerPrediction(){}


    /**
     * @brief getPredictionColor - Generatres color of prediction based on class id
     * @param class_id - class id of prediction
     * @return r, g, b colors for prediction
     */
    prediction_color getPredictionColor(uint8_t class_id);

};


class FrontPrediction
        : public Prediction{

private:
    enum class_ids_names{
        BG=0,
        ROT_FRONT=1,
        TRANS_FRONT=2
    };

public:
    FrontPrediction(std::vector<sensor_msgs::RegionOfInterest> in_boxes,
                    std::vector<int32_t> in_class_ids,
                    std::vector<std::string> in_class_names,
                    std::vector<float_t> in_scores,
                    std::vector<sensor_msgs::Image> in_masks,
                    pcl::PointCloud<pcl::PointXYZRGB> in_cloud):Prediction(in_boxes,
                                                                           in_class_ids,
                                                                           in_class_names,
                                                                           in_scores,
                                                                           in_masks,
                                                                           in_cloud)
    {}

    ~FrontPrediction(){}

    /**
     * @brief getPredictionColor - Generatres color of prediction based on class id
     * @param class_id - class id of prediction
     * @return r, g, b colors for prediction
     */
    prediction_color getPredictionColor(uint8_t class_id);

};

class JointPrediction{

private:
    std::vector<joint_prediction_vertices> predictions;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    /**
     * @brief findRealCoordinatesFromImageCoordinates - find real coordinates in cloud based on image coordinates
     * @param x - x image coordinate
     * @param y - y image coordinate
     * @return point containing real coordinates
     */
    pcl::PointXYZRGB findRealCoordinatesFromImageCoordinates(int x, int y);

public:

    JointPrediction(std::vector<int32_t> in_x1,
                    std::vector<int32_t> in_y1,
                    std::vector<int32_t> in_x2,
                    std::vector<int32_t> in_y2,
                    pcl::PointCloud<pcl::PointXYZRGB> in_cloud);

    ~JointPrediction();

    /**
     * @brief processPrediction - process joint prediction
     * @param output_cloud
     */
    void processPrediction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud);
};

}


