#include "../include/model_builder/prediction.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>



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

    void Prediction::getBoundingBoxInliersIndices(pcl::PointIndices::Ptr boundingbox_inliers_indices,
                                                  int box_y_offset,
                                                  int box_height,
                                                  int box_x_offset,
                                                  int box_width)
    {
        for (int i=box_y_offset; i<=box_y_offset+box_height; i++)
        {
            for (int j=box_x_offset; j<=box_x_offset+box_width; j++)
            {
                boundingbox_inliers_indices->indices.push_back(cloud.width * i + j);
            }
        }
    }

    void Prediction::findPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, bool should_optimize_coeffficients,
                               pcl::SacModel model_type, const int method_type, float distance_threshold,
                               pcl::PointIndices::Ptr plane_inliers, pcl::ModelCoefficients::Ptr plane_coefficients)
    {
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(should_optimize_coeffficients);
        seg.setModelType(model_type);
        seg.setMethodType(method_type);
        seg.setDistanceThreshold(distance_threshold);

        seg.setInputCloud(input_cloud);
        seg.segment(*plane_inliers, *plane_coefficients);
    }

    void Prediction::extractCloudFromIndices(pcl::PointIndices::Ptr indices,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud)
    {
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*extracted_cloud);
    }

    void Prediction::extractCloudFromIndices(pcl::PointIndices::Ptr indices,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud)
    {
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*extracted_cloud);
    }

    Prediction::prediction_color Prediction::getPredictionColor(uint8_t class_id)
    {
        prediction_color color = {0, 0, 0};
        return color;
    }

    void Prediction::computeAverageNormalVector(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                float (&normal)[3])
    {
        double nx = 0;
        double ny = 0;
        double nz = 0;
        for (int i=0; i<cloud_normals->size(); i++)
        {
            nx += cloud_normals->points[i].normal_x;
            ny += cloud_normals->points[i].normal_y;
            nz += cloud_normals->points[i].normal_z;
        }

        nx = float(nx / float(cloud_normals->size()));
        ny = float(ny / float(cloud_normals->size()));
        nz = float(nz / float(cloud_normals->size()));

        // Make sure that vector is normal
        float vector_length = pow(pow(nx, 2.0) + pow(ny, 2.0) + pow(nz, 2.0), 0.5);
        nx = nx / vector_length;
        ny = ny / vector_length;
        nz = nz / vector_length;

        normal[0] = nx;
        normal[1] = ny;
        normal[2] = nz;
    }

    void Prediction::findNormalToPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                                            pcl::PointXYZRGBNormal *normal_line_points,
                                            double radius, int threads_number, int resize_factor)
    {

        if ((resize_factor % 2) != 0)
            resize_factor = resize_factor + 1;

        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne(threads_number);
        ne.setInputCloud(input_cloud);

        // Find normal to plane only in part of the cloud
        std::vector<int> indices;
        for (int i=int(int(resize_factor/2 - 1) *input_cloud->size()/resize_factor);
             i<(int(int(resize_factor/2 + 1) *input_cloud->size()/resize_factor));
             i++)
        {
            indices.push_back(i);
        }
        pcl::IndicesPtr indices_ptr(new std::vector<int> (indices));
        ne.setIndices(indices_ptr);

        // Temporary point cloud to store normals for indices
        pcl::PointCloud<pcl::Normal>::Ptr temp_cloud_normals (new pcl::PointCloud<pcl::Normal>);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        ne.setSearchMethod(tree);

        ne.setRadiusSearch(radius);

        ne.compute(*temp_cloud_normals);

        // Get coordinates of middle point of selected cloud
        normal_line_points->x = input_cloud->points[int(input_cloud->size()/2)].x;
        normal_line_points->y = input_cloud->points[int(input_cloud->size()/2)].y;
        normal_line_points->z = input_cloud->points[int(input_cloud->size()/2)].z;

        // Compute average normal vector
        computeAverageNormalVector(temp_cloud_normals, normal_line_points->normal);
    }

    void Prediction::processPrediction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud,
                                       bool should_find_normal,
                                       std::vector<pcl::PointXYZRGBNormal> &trans_normals_points)
    {
        uint8_t prediction_number = 0;
        for(std::vector<sensor_msgs::RegionOfInterest>::iterator it = boxes.begin(); it != boxes.end(); ++it)
        {
            int box_x_offset = it->x_offset;
            int box_y_offset = it->y_offset;
            int box_width = it->width;
            int box_height = it->height;
            uint8_t class_id = class_ids[prediction_number];
            Prediction::prediction_color color = getPredictionColor(class_id);

            // Get detected bounding box inliers
            pcl::PointIndices::Ptr boundingbox_inliers_indices(new pcl::PointIndices);
            getBoundingBoxInliersIndices(boundingbox_inliers_indices, box_y_offset, box_height, box_x_offset, box_width);

            // Construct cloud from detected bounding box inliers
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_extracted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            extractCloudFromIndices(boundingbox_inliers_indices, bounding_box_extracted_cloud);         

            // Find plane in cloud created from bounding box
            pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
            findPlane(bounding_box_extracted_cloud, true, pcl::SACMODEL_PLANE, pcl::SAC_RANSAC,
                      0.01, plane_indices, plane_coefficients);

            // Construct cloud from detected plane
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            extractCloudFromIndices(plane_indices, bounding_box_extracted_cloud, plane_cloud);

            // Find normal to plane if detected front is translational
            if (should_find_normal && class_id == FrontPrediction::TRANS_FRONT)
            {
                pcl::PointXYZRGBNormal normal_to_plane;
                findNormalToPlane(plane_cloud, &normal_to_plane, 0.05);
                normal_to_plane.r = color.r;
                normal_to_plane.g = color.g;
                normal_to_plane.b = color.b;
                trans_normals_points.push_back(normal_to_plane);
            }

            // Color the detected plane according to its class id
            for (auto &point: plane_cloud->points)
            {
                point.r = color.r;
                point.g = color.g;
                point.b = color.b;
            }
            *output_cloud+=*plane_cloud;

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

    void JointPrediction::processPrediction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud)
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
