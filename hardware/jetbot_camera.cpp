#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <jetson-utils/gstCamera.h>

#include "image_converter.h"

// globals
gstCamera* camera = NULL;

imageConverter* camera_cvt = NULL;
ros::Publisher* camera_pub = NULL;
ros::Publisher* camera_info_pub = NULL;

// aquire and publish camera frame
bool aquireFrame(sensor_msgs::CameraInfo& cam_info_msg)
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

    // populate the Image message
    sensor_msgs::Image img_msg;
    if( !camera_cvt->Convert(img_msg, imageConverter::ROSOutputFormat, imgRGBA) )
    {
        ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
        return false;
    }

    // populate the CameraInfo message
    cam_info_msg.header = img_msg.header;
    cam_info_msg.width = camera->GetWidth();
    cam_info_msg.height = camera->GetHeight();
    // Set camera intrinsic parameters here (example values)
    cam_info_msg.K = {500, 0, cam_info_msg.width / 2.0,
                      0, 500, cam_info_msg.height / 2.0,
                      0, 0, 1};  // Example calibration matrix

    // publish the messages
    camera_pub->publish(img_msg);
    camera_info_pub->publish(cam_info_msg);

    ROS_INFO("published camera frame and camera info");
    return true;
}

// node main loop
int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetbot_camera");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    /*
     * retrieve parameters
     */
    std::string camera_device = "0";	// MIPI CSI camera by default

    private_nh.param<std::string>("device", camera_device, camera_device);

    ROS_INFO("opening camera device %s", camera_device.c_str());


    /*
     * open camera device
     */
    camera = gstCamera::Create(camera_device.c_str());

    if( !camera )
    {
        ROS_ERROR("failed to open camera device %s", camera_device.c_str());
        return 0;
    }

    /*
     * create image converter
     */
    camera_cvt = new imageConverter();

    if( !camera_cvt )
    {
        ROS_ERROR("failed to create imageConverter");
        return 0;
    }

    /*
     * advertise publisher topics
     */
    ros::Publisher camera_publisher = private_nh.advertise<sensor_msgs::Image>("raw", 2);
    ros::Publisher camera_info_publisher = private_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 2);
    camera_pub = &camera_publisher;
    camera_info_pub = &camera_info_publisher;

    /*
     * start the camera streaming
     */
    if( !camera->Open() )
    {
        ROS_ERROR("failed to start camera streaming");
        return 0;
    }

    /*
     * initialize CameraInfo message
     */
    sensor_msgs::CameraInfo camera_info_msg;
    camera_info_msg.header.frame_id = "camera_frame";  // Frame ID for tf
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.D = {0, 0, 0, 0, 0};  // Example distortion coefficients

    /*
     * start publishing video frames
     */
    while( ros::ok() )
    {
        aquireFrame(camera_info_msg);
        ros::spinOnce();
    }

    delete camera;
    return 0;
}
