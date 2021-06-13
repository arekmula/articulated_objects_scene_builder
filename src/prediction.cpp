#include "../include/model_builder/prediction.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <algorithm>



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
        seed = ros::Time::now().toNSec();
        srand(seed);
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
                                       std::vector<pcl::PointXYZRGBNormal> &trans_normals_points,
                                       bool save_separate_clouds,
                                       std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &fronts_point_clouds)
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
                      0.05, plane_indices, plane_coefficients);

            // Construct cloud from detected plane
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            extractCloudFromIndices(plane_indices, bounding_box_extracted_cloud, plane_cloud);

            // Find normal to plane if detected front is translational
            if (should_find_normal && class_id == FrontPrediction::TRANS_FRONT)
            {
                if (plane_cloud->size() > 0)
                {
                    pcl::PointXYZRGBNormal normal_to_plane;
                    findNormalToPlane(plane_cloud, &normal_to_plane, 0.05);
                    normal_to_plane.r = color.r;
                    normal_to_plane.g = color.g;
                    normal_to_plane.b = color.b;
                    trans_normals_points.push_back(normal_to_plane);
                }
            }

            // Color the detected plane according to its class id
            for (auto &point: plane_cloud->points)
            {
                point.r = color.r;
                point.g = color.g;
                point.b = color.b;
            }
            *output_cloud+=*plane_cloud;

            if (save_separate_clouds)
            {
                if (class_id == FrontPrediction::TRANS_FRONT)
                {
                    // Save empty cloud of transitional front only for index matching
                    plane_cloud.reset();
                    fronts_point_clouds.push_back(plane_cloud);
                }
                else if (class_id == FrontPrediction::ROT_FRONT)
                {
                    fronts_point_clouds.push_back(plane_cloud);
                }
            }

            prediction_number++;
        }
    }

    Prediction::prediction_color FrontPrediction::getPredictionColor(uint8_t class_id)
    {

        prediction_color colors={0, 0, 0};
        if (class_id == ROT_FRONT)
        {
            int supplement_color = int(rand() % 128);
            colors.r = supplement_color;
            colors.g = 128 - supplement_color;
            colors.b = 255;
            return colors;
        }
        else if (class_id == TRANS_FRONT)
        {
            int supplement_color = int(rand() % 128);
            colors.r = supplement_color;
            colors.g = 255;
            colors.b = 128 - supplement_color;
            return colors;
        }

        return colors;
    }

    Prediction::prediction_color HandlerPrediction::getPredictionColor(uint8_t class_id)
    {

        Prediction::prediction_color colors={0, 0, 0};
        if (class_id == HANDLER)
        {
            colors.r = 255;
            colors.g = 0;
            colors.b = 0;
            return colors;
        }

        return colors;
    }

    JointPrediction::JointPrediction (std::vector<int32_t> in_x1, std::vector<int32_t> in_y1,
                                      std::vector<int32_t> in_x2, std::vector<int32_t> in_y2,
                                      std::vector<int32_t> in_front_index,
                                      std::vector<int32_t> A,
                                      std::vector<int32_t> B,
                                      std::vector<int32_t> C,
                                      pcl::PointCloud<pcl::PointXYZRGB> in_cloud)
    {
        for (int i=0; i<in_x1.size(); i++)
        {
            rot_joint_prediction_image_vertices prediction = {};
            prediction.x1 = in_x1[i];
            prediction.y1 = in_y1[i];
            prediction.x2 = in_x2[i];
            prediction.y2 = in_y2[i];
            prediction.A = A[i];
            prediction.B = B[i];
            prediction.C = C[i];
            prediction.front_index = in_front_index[i];

            predictions.push_back(prediction);
        }
        cloud = in_cloud;

        std::cout << "Number of joint predictions: " << predictions.size() << std::endl;
    }

    JointPrediction::~JointPrediction()
    {

    }

    float getPointLineDistance(int x, int y, int A, int B, int C)
    {
        float distance_nominator = abs(A * x + B * y + C);
        float distance_denominator = pow((pow(A, 2) + pow(B, 2)), 0.5);

        return float(distance_nominator/distance_denominator);
    }

    float getPointToPointDistance(int x1, int y1, int x2, int y2)
    {
        return pow((pow((x1 - x2), 2) + pow((y1 - y2), 2)), 0.5);
    }

    pcl::PointXYZRGB JointPrediction::findRealCoordinatesFromImageCoordinates(int x, int y, int A, int B, int C)
    {
        pcl::PointXYZRGB point = cloud(x, y);

        // If point is NaN, find nearest one that lands on the joint line and is not NaN.
        if (isnan(point.x))
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            // index the mapping (ordered): cloud_out.points[i] = cloud_in.points[index[i]]
            std::vector<int> indices_mapping = {};
            pcl::removeNaNFromPointCloud(cloud, *temp_out_cloud, indices_mapping);

            // For each existing point get it projection on the image and calculate the distance from the
            // point to joint line and from the point to the input image coordinates
            // Then find the point with minimum distance.
            std::vector<float> distances = {};
            for (int i=0; i<temp_out_cloud->size(); i++)
            {
                int index_in_input_cloud = indices_mapping[i];
                int temp_x = index_in_input_cloud % cloud.width;  // Get the x and y in image coordinates
                int temp_y = floor(index_in_input_cloud/cloud.width);

                float distance_point_to_line = getPointLineDistance(temp_x, temp_y, A, B, C);
                float distance = distance_point_to_line + getPointToPointDistance(x, y, temp_x, temp_y);
                distances.push_back(distance);
            }

            int min_distance_index = std::min_element(distances.begin(), distances.end()) - distances.begin();
            int index_in_input_cloud = indices_mapping[min_distance_index];

            point.x = cloud[index_in_input_cloud].x;
            point.y = cloud[index_in_input_cloud].y;
            point.z = cloud[index_in_input_cloud].z;

        }
        return point;
    }

    bool JointPrediction::findClosestPointInCurrentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                                                         pcl::PointXYZRGB input_point,
                                                         int *output_point_indice,
                                                         int K)
    {
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        if (input_cloud->empty())
            return false;
        kdtree.setInputCloud(input_cloud);

        std::vector<int> nearestPointIndices(K);
        std::vector<float> nearestPointSquaredDistances(K);

        kdtree.nearestKSearch(input_point, K, nearestPointIndices, nearestPointSquaredDistances);
        if (nearestPointIndices.size() < 0 || nearestPointIndices[0] > input_cloud->size() ||
                nearestPointIndices[0] < 0)
        {
            std::cout << "Failed to find nearest point to " << input_point << std::endl;
            // TODO: Add some handling if nearest point couldn't be found
            return false;
        }
        else
        {
            *output_point_indice = nearestPointIndices[0];
            return true;
        }
    }

    void JointPrediction::processPrediction(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> front_separeted_clouds,
                                            std::vector<rot_joint_coordinates> &real_coordinates)
    {
        for (std::vector<rot_joint_prediction_image_vertices>::iterator it = predictions.begin();
             it != predictions.end();
             ++it)
        {
            rot_joint_coordinates current_real_coordinates = {};

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr front_cloud;
            front_cloud = front_separeted_clouds[it->front_index];

            int x1 = it->x1;
            int y1 = it->y1;
            current_real_coordinates.top_point = findRealCoordinatesFromImageCoordinates(x1, y1,
                                                                                         it->A, it->B, it->C);
            int indice = -1;
            bool result = findClosestPointInCurrentCloud(front_cloud, current_real_coordinates.top_point, &indice);
            if (result)
            {
                current_real_coordinates.top_point.x = (*front_cloud)[indice].x;
                current_real_coordinates.top_point.y = (*front_cloud)[indice].y;
                current_real_coordinates.top_point.z = (*front_cloud)[indice].z;
            }
            else
                return;

            int x2 = it->x2;
            int y2 = it->y2;
            current_real_coordinates.bottom_point = findRealCoordinatesFromImageCoordinates(x2, y2,
                                                                                            it->A, it->B, it->C);
            result = findClosestPointInCurrentCloud(front_cloud, current_real_coordinates.bottom_point, &indice);
            if (result)
            {
                current_real_coordinates.bottom_point.x = (*front_cloud)[indice].x;
                current_real_coordinates.bottom_point.y = (*front_cloud)[indice].y;
                current_real_coordinates.bottom_point.z = (*front_cloud)[indice].z;
            }
            else
                return;

            real_coordinates.push_back(current_real_coordinates);
        }
    }
}
