# A brief guideline for the camera calibration

## Step0: Activate the camera node
```python
roscore
rosrun jetbot_ros jetbot_camera
rostopic list # check if the camera node exists
rosrun rqt_image_view rqt_image_view # test the camera view

# Alternative
cd ~/MATSE_jetbot/launch
roslaunch jetbot.launch
```

## Step1: Color correction with pink tint
(ref: https://jonathantse.medium.com/fix-pink-tint-on-jetson-nano-wide-angle-camera-a8ce5fbd797f)
```python
wget https://www.waveshare.com/w/upload/e/eb/Camera_overrides.tar.gz  # Download the tuning profile for Jetson 4.2

tar zxvf Camera_overrides.tar.gz  # unzip the profile

sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/  # Copy the profile to the camera firmware directory

sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp

sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
```

## Step2: Calibrate the camera using camera_calibration
(ref: https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
```python
rosdep install camera_calibration  # getting the dependencies and compiling the driver
rosrun camera_calibration cameracalibrator.py --size 8x10 --square 0.2 image:=/camera/image_raw camera:=/camera
```

## Step3: Apply the calibration using image_proc
(ref: https://wiki.ros.org/image_proc#image_proc.2Fcturtle.Mini_Tutorial)
```python
ROS_NAMESPACE=camera rosrun image_proc image_proc  # activate the image_proc node for rectification
```
