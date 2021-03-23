// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "detection_msgs/FrontPrediction.h"
#include "detection_msgs/HandlerPrediction.h"

namespace model_builder {

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
    void processPrediction(pcl::PointCloud<pcl::PointXYZRGB> *output_cloud);

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

}
