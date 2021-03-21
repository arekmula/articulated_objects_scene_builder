# articulated_objects_scene_builder

## About
ROS node that builds point cloud-based model of the scene enhanced with information about articulated objects. It subscribes to following topics:
- `hz_points` which contains input point cloud to process
- `front_prediction` which contains information about detected fronts of articulated objects. [See node](https://github.com/arekmula/ros_front_detection_segmentation)
- `handler_prediction_topic` which contains information about detected handlers of articulated objects. [See node](https://github.com/arekmula/ros_handler_detector)
- `joint_prediction_topic` which contains information about detected joints of articulated objects that are rotational. [See node](https://github.com/arekmula/ros_joint_segmentation)

The node publish following topics:
- `image_to_process` - RGB image obtained from input point cloud, which can be processed by external nodes
- `currently_processed_point_cloud` - currently processed point cloud
- `processed_point_cloud` - processed point cloud with marked data

## Dependencies
- ROS Noetic
- PCL library `sudo apt install libpcl-dev`

## Run with
```
rosrun model_builder model_builder_node
```
