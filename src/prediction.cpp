#include "../include/model_builder/prediction.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>


namespace model_builder{

    Prediction::Prediction(std::vector<sensor_msgs::RegionOfInterest> in_boxes,
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

        std::cout << "Number of predictions: " << boxes.size() << std::endl;
    }

    Prediction::~Prediction()
    {

    }

    pcl::PointXYZRGB Prediction::findRealCoordinatesFromImageCoordinates(int x, int y)
    {
        pcl::PointXYZRGB point = cloud(x, y);

        // If point not existing in point cloud, find nearest one
        int loop_count = 1;
        while (isnan(point.x))
        {
            // Values to move. Multiplied by loop_count
            std::vector<int> dx = { 0, 1, 0, -1 };
            std::vector<int> dy = { 1, 0, -1, 0 };
            std::transform(dx.begin(), dx.end(), dx.begin(), std::bind1st(std::multiplies<int>(), loop_count));
            std::transform(dy.begin(), dy.end(), dy.begin(), std::bind1st(std::multiplies<int>(), loop_count));

            // TODO: If x+dx[i] or y+dy[i] are out of width or height, abort coordinates
            for (int i=0; isnan(point.x) && i < 4; ++i)
            {
                point = cloud(x+ dx[i], y + dy[i]);
            }
            loop_count++;
        }

        return point;
    }


    Prediction::prediction_color Prediction::getPredictionColor(uint8_t class_id)
    {
        prediction_color color = {0, 0, 0};
        return color;
    }

    void Prediction::processPrediction(pcl::PointCloud<pcl::PointXYZRGB> *output_cloud)
    {

        uint8_t HANDLER_BLUE_COLOR=255;
        uint8_t prediction_number = 0;
        for(std::vector<sensor_msgs::RegionOfInterest>::iterator it = boxes.begin(); it != boxes.end(); ++it)
        {
            int box_x_offset = it->x_offset;
            int box_y_offset = it->y_offset;
            int box_width = it->width;
            int box_height = it->height;
            uint8_t class_id = class_ids[prediction_number];
            Prediction::prediction_color color = getPredictionColor(class_id);

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
            pcl::PointIndices::Ptr boundingbox_inliers(new pcl::PointIndices);

            // Construct bounding box inliers. The point cloud is stored in 1D organized array
            for (int i=box_y_offset; i<=box_y_offset+box_height; i++)
            {
                for (int j=box_x_offset; j<=box_x_offset+box_width; j++)
                {
                    boundingbox_inliers->indices.push_back(cloud.width * i + j);
                }
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract_bounding_box;
            extract_bounding_box.setInputCloud(cloud.makeShared());
            extract_bounding_box.setIndices(boundingbox_inliers);
            extract_bounding_box.setNegative(false);
            extract_bounding_box.filter(*extracted_cloud);

            for (auto &point: output_cloud->points)
            {
                if (point.x > bottom_left.x && point.y > bottom_left.y
                        && point.x < top_right.x && point.y < top_right.y && point.b != HANDLER_BLUE_COLOR)
                {
                        point.r = color.r;
                        point.g = color.g;
                        point.b = color.b;
                }
            }

            prediction_number++;
        }
    }

    Prediction::prediction_color FrontPrediction::getPredictionColor(uint8_t class_id)
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

    Prediction::prediction_color HandlerPrediction::getPredictionColor(uint8_t class_id)
    {

        long int seed = ros::Time::now().toNSec();
        srand(seed);
        Prediction::prediction_color colors={0, 0, 0};
        if (class_id == HANDLER)
        {
            colors.r = 0;
            colors.g = 0;
            colors.b = 255;
            return colors;
        }

        return colors;
    }

    JointPrediction::JointPrediction (std::vector<int32_t> in_x1, std::vector<int32_t> in_y1,
                                      std::vector<int32_t> in_x2, std::vector<int32_t> in_y2,
                                      pcl::PointCloud<pcl::PointXYZRGB> in_cloud)
    {
        for (int i=0; i<in_x1.size(); i++)
        {
            joint_prediction_vertices prediction = {};
            prediction.x1 = in_x1[i];
            prediction.y1 = in_y1[i];
            prediction.x2 = in_x2[i];
            prediction.y2 = in_y2[i];

            predictions.push_back(prediction);
        }
        cloud = in_cloud;

        std::cout << "Number of joint predictions: " << predictions.size() << std::endl;
    }

    JointPrediction::~JointPrediction()
    {

    }

    pcl::PointXYZRGB JointPrediction::findRealCoordinatesFromImageCoordinates(int x, int y)
    {
        pcl::PointXYZRGB point = cloud(x, y);

        // If point not existing in point cloud, find nearest one
        int loop_count = 1;
        while (isnan(point.x))
        {
            // Values to move. Multiplied by loop_count
            std::vector<int> dx = { 0, 1, 0, -1 };
            std::vector<int> dy = { 1, 0, -1, 0 };
            std::transform(dx.begin(), dx.end(), dx.begin(), std::bind1st(std::multiplies<int>(), loop_count));
            std::transform(dy.begin(), dy.end(), dy.begin(), std::bind1st(std::multiplies<int>(), loop_count));

            // TODO: If x+dx[i] or y+dy[i] are out of width or height, abort coordinates
            for (int i=0; isnan(point.x) && i < 4; ++i)
            {
                point = cloud(x+ dx[i], y + dy[i]);
            }
            loop_count++;
        }

        return point;
    }

    void JointPrediction::processPrediction(pcl::PointCloud<pcl::PointXYZRGB> *output_cloud)
    {
        for (std::vector<joint_prediction_vertices>::iterator it = predictions.begin(); it != predictions.end(); ++it)
        {
            int x1 = it->x1;
            int y1 = it->y1;
            pcl::PointXYZRGB top_point = findRealCoordinatesFromImageCoordinates(x1, y1);

            int x2 = it->x2;
            int y2 = it->y2;
            pcl::PointXYZRGB bottom_point = findRealCoordinatesFromImageCoordinates(x2, y2);

            for (auto &point: output_cloud->points)
            {
                if (point.x > bottom_point.x && point.y > bottom_point.y
                        && point.x < top_point.x+0.01 && point.y < top_point.y+0.01)
                {
                        point.r = 255;
                        point.g = 255;
                        point.b = 0;
                }
            }
        }
    }
}
