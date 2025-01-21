#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jetson-utils/gstCamera.h>
#include "image_converter.h"
#include <camera_info_manager/camera_info_manager.h>  // 添加的头文件


// globals    
gstCamera* camera = NULL;
imageConverter* camera_cvt = NULL;
ros::Publisher* camera_pub = NULL;
ros::Publisher* camera_info_pub = NULL;  // 新增一个发布器用于发布 camera_info

camera_info_manager::CameraInfoManager* camera_info_manager_ = NULL;  // 用于管理 CameraInfo
sensor_msgs::CameraInfo camera_info_msg;  // 用于存储 camera_info 消息


// acquire and publish camera frame
bool aquireFrame()
{
    float4* imgRGBA = NULL;

    // get the latest frame
    if( !camera->CaptureRGBA((float**)&imgRGBA, 1000) )
    {
        ROS_ERROR("failed to capture camera frame");
        return false;
    }

    // assure correct image size
    if( !camera_cvt->Resize(camera->GetWidth(), camera->GetHeight(), IMAGE_RGBA32F) )
    {
        ROS_ERROR("failed to resize camera image converter");
        return false;
    }

    // populate the image message
    sensor_msgs::Image msg;
    if( !camera_cvt->Convert(msg, imageConverter::ROSOutputFormat, imgRGBA) )
    {
        ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
        return false;
    }

    // publish the image message
    camera_pub->publish(msg);
    ROS_INFO("published camera frame");

    // also publish camera_info
    camera_info_msg.header.stamp = ros::Time::now();
    camera_info_msg.header.frame_id = "camera_link";  // 假设相机的坐标系为 camera_link
    camera_info_pub->publish(camera_info_msg);  // 发布 camera_info
    return true;
}

// node main loop
int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetbot_camera");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // retrieve parameters
    std::string camera_device = "0";    // MIPI CSI camera by default
    private_nh.param<std::string>("device", camera_device, camera_device);
    ROS_INFO("opening camera device %s", camera_device.c_str());

    // open camera device
    camera = gstCamera::Create(camera_device.c_str());
    if( !camera )
    {
        ROS_ERROR("failed to open camera device %s", camera_device.c_str());
        return 0;
    }

    // create image converter
    camera_cvt = new imageConverter();
    if( !camera_cvt )
    {
        ROS_ERROR("failed to create imageConverter");
        return 0;
    }

    // advertise publisher topics
    ros::Publisher camera_publisher = private_nh.advertise<sensor_msgs::Image>("raw", 2);
    camera_pub = &camera_publisher;

    // advertise camera_info topic
    ros::Publisher camera_info_publisher = private_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 2);
    camera_info_pub = &camera_info_publisher;

    // create camera_info_manager and load camera calibration parameters from YAML file
    std::string camera_info_url = "/home/jetbot/workspace/catkin_ws/src/jetbot_ros/ost.yaml";  // 直接使用绝对路径
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh, "camera", camera_info_url);
    if (camera_info_manager_->isCalibrated())
    {
        camera_info_msg = camera_info_manager_->getCameraInfo();
        ROS_INFO("Camera calibration loaded successfully.");
    }
    else
    {
        ROS_WARN("Camera calibration file not found.");
    }

    if (camera_info_manager_->isCalibrated())
    {
        camera_info_msg = camera_info_manager_->getCameraInfo();
        ROS_INFO("Camera calibration loaded successfully.");
    }
    else
    {
        ROS_WARN("Camera calibration file not found.");
    }

    // start the camera streaming
    if( !camera->Open() )
    {
        ROS_ERROR("failed to start camera streaming");
        return 0;
    }

    // start publishing video frames
    while( ros::ok() )
    {
        //if( raw_pub->getNumSubscribers() > 0 )
        aquireFrame();

        ros::spinOnce();
    }

    delete camera;
    delete camera_info_manager_;
    return 0;
}
