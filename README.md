# Sphero RVR Robot Detection

## Introduction
This code provides a real-time object detection and position estimation solution using the Oak-D camera and DepthAI API. The code detects objects using a YOLOv5 neural network and publishes the detected objects' positions and bounding boxes to specific ROS topics.

## Prerequisites
- Ubuntu 20.04, should also work with Debian 10 but not guaranteed
- Python 3.8.10 or above
- OpenCV
- DepthAI
- DepthAI SDK
- ROS Noetic

## Installation
1. Install DepthAI:
   ```
   sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
   ```
2. Install DepthAI SDK:
   ```
   python3 -m pip install depthai-sdk
   ```
3. Install ROS Noetic by following the instructions on:
   http://wiki.ros.org/noetic/Installation/Ubuntu
4. OpenCV should be installed by default in Ubuntu 20.04, if not:
   ```
   sudo apt update
   ```
   ```
   sudo apt install libopencv-dev python3-opencv
   ```
## Usage
1. The main code is `oak_detector/oak_detector.py` and it can be run with the following command:
    ```
    python3 ./oak_detector/oak_detector.py ./models/rvr_stretched_1.blob ./models/rvr_stretched_1.json [visualize]
    ```
    - General usage: `Python3 /path/oak_detector.py /path/blob_file_name /path/json_file_name`
    - `/path/to/blob_file_name`: Path to a blob file containing the weights of the neural network model.
    - `/path/to/json_file_name`: Path to the corresponding JSON file containing the configuration for the neural network.
    - `visualize` (optional): Add this argument to enable real-time visualization of the detected robots.
2. The code will start the Oak-D camera, perform object detection, and publish the detected robots bounding boxes to the ROS topic named `/oak` and positions to the ROS topic `/tf`.
3. There is 3 pre-trained neural network detecting RVR robots in the `models` folder.
4. The most suitable model among the 3 for robot detection to have detection on full FOV of the Oak-D is `rvr_stretched_1.blob`
## ROS Topics
The code publishes the following ROS topics:
- `/oak` (vision_msgs/Detection2DArray): Contains the detection information for the detected robots: bounding boxes, confidence, object type.
- `/tf`  (tf/tfMessage): Contains the position (x,y) of detected robots.

## Other functionalities

1. To retrieve a trained model with roboflow:
   ```
   Python3 ./utils/depthai_sdk.py roboflow_model_name
   ```
   - `roboflow_model_name`: name of a pre-trained roboflow model, example: rvr_detection_v2/1
   - If there is an error, it probably means that the API key in the script reached the limit of downloads, you must change it by your own key, sign in to Roboflow to get one.

2. `aruco_detector/script_calibration.py` outputs calibration matrix and distorsion coefficients that are need for `aruco_detector/aruco_position_detection.py`. You need first to have 20 non-redundant images taken by the camera you want to calibrate.
   ```
   python3 aruco_detector/script_calibration.py
   ```
3. To run aruco detection:
   ```
   python3 aruco_detector/aruco_position_detection.py
   ```
4. `utils/record.py` records videos with Oak-D camera:
   ```
   Python3 ./utils/record.py output_file_name
   ```
   - `output_file_name`: name of the output video
5. `utils/sampling_vods.py` samples recorded videos to keep only some images:
   ```
   Python3 ./utils/sampling_vods.py
   ```
6. `utils/result_experiment.py` Script that compares two different position estimations stored in rosbags in csv format, this was useful to assess the precision of the software in a real experiment:
   ```
   Python3 ./utils/result_experiment.py
   ```
7. `pytorch_to_blob` converts a NN pytorch file to a blob file runnable directly on Oak-D:
   ```
   Python3 ./utils/pytoch_to_blob pytorch_file_path
   ```
   - This script is only useful if you want to convert a pytorch file, it requires the installation of pytorch but since this a heavy requirement, install it only if you are going to run this specific script:
   ```
   pip3 install torch==1.9.1+cpu torchvision==0.10.1+cpu -f https://download.pytorch.org/whl/torch_stable.html
   ```
   
