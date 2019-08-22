// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "RealSense2Engine.h"

#include "../ORUtils/FileUtils.h"

#include <cstdio>
#include <stdexcept>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <unistd.h>

#ifdef COMPILE_WITH_RealSense2

#define tt 1000.0

static queue<DataReader::IMUData> related_imu;
rs2::frame_queue depth_queue(1);
rs2::frame_queue color_queue(1);
mutex get_imu_lock;
static bool default_depth2color = false; // 这里估计也是计算出来的,而且用的cpu,需要12ms


using namespace InputSource;
using namespace ITMLib;

static void print_device_information(const rs2::device& dev)
{
	// Each device provides some information on itself
	// The different types of available information are represented using the "RS2_CAMERA_INFO_*" enum
	
	std::cout << "Device information: " << std::endl;
	//The following code shows how to enumerate all of the RS2_CAMERA_INFO
	//Note that all enum types in the SDK start with the value of zero and end at the "*_COUNT" value
	for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
	{
		rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
		//SDK enum types can be streamed to get a string that represents them
		std::cout << "  " << std::left << std::setw(20) << info_type << " : ";
		
		//A device might not support all types of RS2_CAMERA_INFO.
		//To prevent throwing exceptions from the "get_info" method we first check if the device supports this type of info
		if (dev.supports(info_type))
			std::cout << dev.get_info(info_type) << std::endl;
		else
			std::cout << "N/A" << std::endl;
	}
}


RealSense2Engine::RealSense2Engine(const char *calibFilename, bool alignColourWithDepth,
								   Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
: BaseImageSourceEngine(calibFilename)
{
	this->calib.disparityCalib.SetStandard();
	this->calib.trafo_rgb_to_depth = ITMExtrinsics();
	this->calib.intrinsics_d = this->calib.intrinsics_rgb;
	
	this->imageSize_d = requested_imageSize_d;
	this->imageSize_rgb = requested_imageSize_rgb;

    //init delay
    collect_frame_num = 0;
	last_image_time = 0.0f;
	//IMAGE DELAY 2
    inputRGBImage1 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRGBImage2 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRGBImage3 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRawDepthImage1 = new ITMShortImage(this->imageSize_d, true, true);
    inputRawDepthImage2 = new ITMShortImage(this->imageSize_d, true, true);
    inputRawDepthImage3 = new ITMShortImage(this->imageSize_d, true, true);
	
	this->ctx = std::unique_ptr<rs2::context>(new rs2::context());
	
	rs2::device_list availableDevices = ctx->query_devices();

	printf("There are %d connected RealSense devices.\n", availableDevices.size());
	if (availableDevices.size() == 0) {
		dataAvailable = false;
		ctx.reset();
		return;
	}
	
	this->device = std::unique_ptr<rs2::device>(new rs2::device(availableDevices.front()));
	
	print_device_information(*device);

    // Check if advanced-mode is enabled to pass the custom config
    auto advanced_mode_dev = this->device->as<rs400::advanced_mode>();
    if (!advanced_mode_dev.is_enabled())
    {
        // If not, enable advanced-mode
        advanced_mode_dev.toggle_advanced_mode(true);
        cout << "Advanced mode enabled. " << endl;
    }
    std::ifstream t("/home/zhuzunjie/Projects/D435i/DepthBetterPerformance_4_26.json");
    std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    advanced_mode_dev.load_json(preset_json);

	this->pipe = std::unique_ptr<rs2::pipeline>(new rs2::pipeline(*ctx));



	rs2::config config;
	config.enable_stream(RS2_STREAM_DEPTH, imageSize_d.x, imageSize_d.y, RS2_FORMAT_Z16, 30);
	config.enable_stream(RS2_STREAM_COLOR, imageSize_rgb.x, imageSize_rgb.y, RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
    config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
    config.enable_stream(RS2_STREAM_INFRARED, 1);
    config.enable_stream(RS2_STREAM_INFRARED, 2);

    // get depth scale
	auto availableSensors = device->query_sensors();
	std::cout << "Device consists of " << availableSensors.size() << " sensors:" << std::endl;
	for (rs2::sensor sensor : availableSensors) {
		//print_sensor_information(sensor);
		
		if (rs2::depth_sensor dpt_sensor = sensor.as<rs2::depth_sensor>()) {
			float scale = dpt_sensor.get_depth_scale();
			std::cout << "Scale factor for depth sensor is: " << scale << std::endl;
			this->calib.disparityCalib.SetFrom(scale, 0, ITMLib::ITMDisparityCalib::TRAFO_AFFINE);
		}
	}

	//get imu
    static double gtime=0.0,atime=0.0,accel1,accel2,accel3;
    static DataReader::IMUData imu(0,0,0,0,0,0,0);
    auto motion_callback = [&](const rs2::frame& frame)
    {
        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
//            std::lock_guard<std::mutex> lock(get_image_lock);
            color_queue(fs.get_color_frame());
            depth_queue(fs.get_depth_frame());
        }
        if(auto motion = frame.as<rs2::motion_frame>())
        {
            if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL)
            {
                double t = motion.get_timestamp() * tt;
                // Get accelerometer measures
                rs2_vector accel_data = motion.get_motion_data();

                if(!(gtime < 1e-3 || t < gtime || atime >= gtime)){
                    imu._a[0] = ((t-gtime)*accel1+(gtime-atime)*accel_data.x)/(t-atime);
                    imu._a[1] = ((t-gtime)*accel2+(gtime-atime)*accel_data.y)/(t-atime);
                    imu._a[2] = ((t-gtime)*accel3+(gtime-atime)*accel_data.z)/(t-atime);
                    std::lock_guard<std::mutex> lock(get_imu_lock);
                    related_imu.push(imu);
                }
                atime = t;
                accel1 = accel_data.x;
                accel2 = accel_data.y;
                accel3 = accel_data.z;
            }

            if (motion.get_profile().stream_type() == RS2_STREAM_GYRO)
            {
                // Get the timestamp of the current frame
                double t = motion.get_timestamp() * tt;
                // Get gyro measures
                rs2_vector gyro_data = motion.get_motion_data();

                gtime = t;
                imu._t = t;
                imu._g[0] = gyro_data.x;
                imu._g[1] = gyro_data.y;
                imu._g[2] = gyro_data.z;
            }
        }
    };

	rs2::pipeline_profile pipeline_profile = pipe->start(config, motion_callback);

	//set intrinsics and extrinsics
	rs2::video_stream_profile depth_stream_profile = pipeline_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	rs2::video_stream_profile color_stream_profile = pipeline_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

	rs2_intrinsics intrinsics_depth = depth_stream_profile.get_intrinsics();
	rs2_intrinsics intrinsics_rgb = color_stream_profile.get_intrinsics();
    this->calib.intrinsics_rgb.SetFrom(this->imageSize_rgb[0], this->imageSize_rgb[1], intrinsics_rgb.fx, intrinsics_rgb.fy, intrinsics_rgb.ppx, intrinsics_rgb.ppy);
    this->calib.intrinsics_d.SetFrom(this->imageSize_d[0], this->imageSize_d[1], intrinsics_depth.fx, intrinsics_depth.fy, intrinsics_depth.ppx, intrinsics_depth.ppy);

	rs2_extrinsics rs_extrinsics = depth_stream_profile.get_extrinsics_to(color_stream_profile);
	Matrix4f extrinsics;
	extrinsics.m00 = rs_extrinsics.rotation[0]; extrinsics.m10 = rs_extrinsics.rotation[1]; extrinsics.m20 = rs_extrinsics.rotation[2];
	extrinsics.m01 = rs_extrinsics.rotation[3]; extrinsics.m11 = rs_extrinsics.rotation[4]; extrinsics.m21 = rs_extrinsics.rotation[5];
	extrinsics.m02 = rs_extrinsics.rotation[6]; extrinsics.m12 = rs_extrinsics.rotation[7]; extrinsics.m22 = rs_extrinsics.rotation[8];
	extrinsics.m30 = rs_extrinsics.translation[0];
	extrinsics.m31 = rs_extrinsics.translation[1];
	extrinsics.m32 = rs_extrinsics.translation[2];
	extrinsics.m33 = 1.0f;
	extrinsics.m03 = 0.0f; extrinsics.m13 = 0.0f; extrinsics.m23 = 0.0f;
	this->calib.trafo_depth_to_rgb.SetFrom(extrinsics);
}

RealSense2Engine::~RealSense2Engine()
{
	if (pipe) {
		pipe->stop();
	}
}

void RealSense2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	dataAvailable = false;

	rs2::frame color,depth;
	depth_queue.try_wait_for_frame(&depth);
    color_queue.try_wait_for_frame(&color);

    collect_frame_num++;
    imgtime = depth.get_timestamp() * tt;
    if(last_image_time < 1e-3) last_image_time = imgtime;

	//execute image delay
    inputRGBImage1->SetFrom(inputRGBImage2, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    inputRGBImage2->SetFrom(inputRGBImage3, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    inputRawDepthImage1->SetFrom(inputRawDepthImage2, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
    inputRawDepthImage2->SetFrom(inputRawDepthImage3, ORUtils::MemoryBlock<short>::CPU_TO_CPU);

	constexpr size_t rgb_pixel_size = sizeof(Vector4u);
	static_assert(4 == rgb_pixel_size, "sizeof(rgb pixel) must equal 4");
	const Vector4u * color_frame = reinterpret_cast<const Vector4u*>(color.get_data());
	
	constexpr size_t depth_pixel_size = sizeof(uint16_t);
	static_assert(2 == depth_pixel_size, "sizeof(depth pixel) must equal 2");
	auto depth_frame = reinterpret_cast<const uint16_t *>(depth.get_data());
	
	// setup infinitam frames
    Vector4u *rgb = inputRGBImage3->GetData(MEMORYDEVICE_CPU);
    short *rawDepth = inputRawDepthImage3->GetData(MEMORYDEVICE_CPU);
    inputRGBImage3->Clear(); inputRawDepthImage3->Clear();

    // Let's just memcpy the data instead of using loops
	::memcpy(rgb, color_frame, rgb_pixel_size * rgbImage->noDims.x*rgbImage->noDims.y);
	::memcpy(rawDepth, depth_frame, depth_pixel_size * rawDepthImage->noDims.x * rawDepthImage->noDims.y);

    if(collect_frame_num >= 3){
        rgbImage->SetFrom(inputRGBImage1, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        rawDepthImage->SetFrom(inputRawDepthImage1, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
    } else if(collect_frame_num == 2){
        rgbImage->SetFrom(inputRGBImage2, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        rawDepthImage->SetFrom(inputRawDepthImage2, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
    } else if(collect_frame_num == 1){
        rgbImage->SetFrom(inputRGBImage3, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        rawDepthImage->SetFrom(inputRawDepthImage3, ORUtils::MemoryBlock<short>::CPU_TO_CPU);

        cout<< "first" << related_imu.size() <<endl;
        std::lock_guard<std::mutex> lock(get_imu_lock);
        while(related_imu.front()._t <= imgtime)
            related_imu.pop();
    }

    cv::Mat gray = cv::Mat::zeros(480,640,CV_8UC1);
    for(int i=0;i<480;i++){
        int id = i*640;
        for(int j=0;j<640;j++) {
            int id1 = id + j;
            gray.at<uchar>(i, j) = color_frame[id1].x * 0.114 + color_frame[id1].y * 0.587 + color_frame[id1].z * 0.2989;
        }
    }
    gray.copyTo(grayimg);

	dataAvailable = true;
}

bool RealSense2Engine::hasMoreImages(void) const {
	return pipe != nullptr;
}

Vector2i RealSense2Engine::getDepthImageSize(void) const {
	return pipe ? imageSize_d : Vector2i(0,0);
}

Vector2i RealSense2Engine::getRGBImageSize(void) const {
	return pipe ? imageSize_rgb : Vector2i(0,0);
}

void RealSense2Engine::getRelatedIMU(vector<DataReader::IMUData> &relatedIMU)
{
    std::lock_guard<std::mutex> lock(get_imu_lock);
    if(related_imu.front()._t <= last_image_time)
        related_imu.pop();
    cout<< "related" << related_imu.size()<<endl;

    while(!related_imu.empty()) {
        if(related_imu.front()._t <= imgtime)
        {
            /*cout << related_imu.front()._t << endl;
            cout << related_imu.front()._a[0] << " " << related_imu.front()._a[1] << " " << related_imu.front()._a[1] << " "
                 << related_imu.front()._g[0] << " " << related_imu.front()._g[1] << " " << related_imu.front()._g[2] << endl;*/
            relatedIMU.push_back(related_imu.front());
            related_imu.pop();
        }
        else break;
    }

    last_image_time = imgtime;
    cout << "current: " << relatedIMU.size()<< endl;
}

#else

using namespace InputSource;

RealSense2Engine::RealSense2Engine(const char *calibFilename, bool alignColourWithDepth,
								   Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
: BaseImageSourceEngine(calibFilename)
{
	printf("compiled without RealSense SDK 2.X support\n");
}
RealSense2Engine::~RealSense2Engine()
{}
void RealSense2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool RealSense2Engine::hasMoreImages(void) const
{ return false; }
Vector2i RealSense2Engine::getDepthImageSize(void) const
{ return Vector2i(0,0); }
Vector2i RealSense2Engine::getRGBImageSize(void) const
{ return Vector2i(0,0); }
void RealSense2Engine::getRelatedIMU(vector<DataReader::IMUData> &relatedIMU)
{}

#endif

