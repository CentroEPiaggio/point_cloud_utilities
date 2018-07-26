# Point Cloud Utilities

A package with utilities for Point Clouds.
Contains:
1) a ROS node to downsample a generic resolution point cloud (sensor_msgs::PointCloud2) taken from a specified topic (e.g. */kinect2/sd/points*) to a any lower resolution (*x_res * y_res*) keeping the PointCloud2 organized (THIS IS IMPORTANT ECTO (in SOMA) TO WORK - PCL DOES NOT HAVE ANY ORGANIZED DOWNSAMPLING FILTER). The node publishes the resulting cloud (sensor_msgs::PointCloud2) to another specified topic.
2) a ROS node to save the point cloud as PCD. A point cloud (sensor_msgs::PointCloud2) taken from a specified topic (e.g. */kinect2/sd/points*) is saved to a PCD file.
2) a ROS node to crop the point cloud centrally to a any lower resolution (*x_res * y_res*) keeping the PointCloud2 organized. The node publishes the resulting cloud (sensor_msgs::PointCloud2) to another specified topic.

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

The only dependencies are ROS Indigo or newer and PCL.

### Installing

To install this package just clone and catkin build.

## Running the downsampler 

### For UNIPI setup (Kinect One) 

Launch the pc_downsampler node (this downsamples the point cloud taken from */kinect2/sd/points* to the resolution specified as param in the launch and publishes to */down_sampled_points*).

```
roslaunch point_cloud_utilities pc_downsampler_kinect2.launch
```

### For other setups

Run the following commands on separate terminals:

```
roscore
```

```
rosparam set x_res (desired_x_resolution) && rosparam set y_res (desired_y_resolution)
```

```
rosrun point_cloud_utilities pc_downsampler input_topic:=/your_input_topic output_topic:=/your_output_topic
```

Or please create a new launch file.

### To set different output resolution

Modify the following lines in *src/main.cpp* :

```
int x_res = 320;
int y_res = 240;
```

This might be modified in a following version.

## Running the PCD saver

### For UNIPI setup (Kinect One) 

Launch the pc_to_pcd node (this saves the point cloud taken from */kinect2/sd/points* to a test_pcd.pcd file *~/.ros*).

```
roslaunch point_cloud_utilities pc_to_pcd_kinect2.launch
```

## Running the cropper

### For UNIPI setup (Kinect One) 

Launch the pc_cropper node (this crops the point cloud taken from */kinect2/sd/points* to the dimensions specified as param in the launch (x_res, y_res) and publishes to */cropped_points*).

```
roslaunch point_cloud_utilities pc_cropper_kinect2.launch
```

## Running the PCD transformer

Launch the pcd_transform node (this starts a service called */pcd_transform_service*)).

```
roscore
```

```
rosrun point_cloud_utilities point_cloud_utilities_transfrom
```

Then call the service on a seperate terminal specifying the path to the .pcd file and the transformation:

```
rosservice call /pcd_transform_service "pcd_path: './file.pcd'
transform:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

```

## Other launch files

Other useful launch files are also provided to combine dowsampling, cropping and saving.