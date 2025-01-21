/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <camera_calibration_parsers/parse_yml.h>

#include <jetson-utils/gstCamera.h>

#include "image_converter.h"



// globals	
gstCamera* camera = NULL;

imageConverter* camera_cvt = NULL;
ros::Publisher* camera_pub = NULL;


// aquire and publish camera frame
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

//	// populate the message
//	sensor_msgs::Image msg;
//
//	if( !camera_cvt->Convert(msg, imageConverter::ROSOutputFormat, imgRGBA) )
//	{
//		ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
//		return false;
//	}
//
//	// publish the message
//	camera_pub->publish(msg);
//	ROS_INFO("published camera frame");
//	return true;

    // populate the message
	sensor_msgs::Image msg;
	sensor_msgs::CameraInfo cam_info_msg;
	std::string camera_name;
	std::string calib_path;
	calib_path = "/home/jetbot/workspace/catkin_ws/src/jetbot_ros/ost.yaml";
	camera_name = "narrow_stereo";
    camera_calibration_parsers::readCalibrationYml(
    calib_path, camera_name, cam_info_msg);

	//ROS_INFO(cam_info_msg);

	if( !camera_cvt->Convert(msg, sensor_msgs::image_encodings::BGR8, imgRGBA) )
	{
		ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
		return false;
	}

	// header
	static uint32_t seq = 0;
	static const char* frame_id = camera->GetResource().c_str();
	msg.header.seq = seq;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = frame_id;
	seq++;

	cam_info_msg.header.frame_id = msg.header.frame_id;
    cam_info_msg.header.stamp = msg.header.stamp;
	// publish the message
	camera_pub->publish(msg, cam_info_msg);
	ROS_DEBUG("published camera frame");
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
	camera_pub = &camera_publisher;


	/*
	 * start the camera streaming
	 */
	if( !camera->Open() )
	{
		ROS_ERROR("failed to start camera streaming");
		return 0;
	}


	/*
	 * start publishing video frames
	 */
	while( ros::ok() )
	{
		//if( raw_pub->getNumSubscribers() > 0 )
			aquireFrame();

		ros::spinOnce();
	}

	delete camera;
	return 0;
}
