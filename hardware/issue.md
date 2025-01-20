CMakeFiles/jetbot_camera.dir/src/jetbot_camera.cpp.o: In function `aquireFrame()':
jetbot_camera.cpp:(.text+0x3cc): undefined reference to `camera_calibration_parsers::readCalibrationYml(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >&, sensor_msgs::CameraInfo_<std::allocator<void> >&)'
jetbot_camera.cpp:(.text+0x640): undefined reference to `image_transport::CameraPublisher::publish(sensor_msgs::Image_<std::allocator<void> > const&, sensor_msgs::CameraInfo_<std::allocator<void> > const&) const'
CMakeFiles/jetbot_camera.dir/src/jetbot_camera.cpp.o: In function `main':
jetbot_camera.cpp:(.text+0xf54): undefined reference to `image_transport::ImageTransport::ImageTransport(ros::NodeHandle const&)'
jetbot_camera.cpp:(.text+0xf8c): undefined reference to `image_transport::ImageTransport::advertiseCamera(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, unsigned int, bool)'
jetbot_camera.cpp:(.text+0x1190): undefined reference to `image_transport::ImageTransport::~ImageTransport()'
jetbot_camera.cpp:(.text+0x1398): undefined reference to `image_transport::ImageTransport::~ImageTransport()'
collect2: error: ld returned 1 exit status
jetbot_ros/CMakeFiles/jetbot_camera.dir/build.make:167: recipe for target '/home/jetbot/workspace/catkin_ws/devel/lib/jetbot_ros/jetbot_camera' failed
make[2]: *** [/home/jetbot/workspace/catkin_ws/devel/lib/jetbot_ros/jetbot_camera] Error 1
CMakeFiles/Makefile2:615: recipe for target 'jetbot_ros/CMakeFiles/jetbot_camera.dir/all' failed
make[1]: *** [jetbot_ros/CMakeFiles/jetbot_camera.dir/all] Error 2
Makefile:140: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j4 -l4" failed

