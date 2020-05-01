//
// Created by zhuzunjie on 2019/8/24.
//

// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "AzurekinectEngine.h"

#include <cstdio>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <mutex>

#include "../ORUtils/FileUtils.h"
#include "../ORUtils/Stopwatch.h"

#ifdef COMPILE_WITH_Azurekinect

using namespace InputSource;
using namespace ITMLib;

pinhole_t AzureKinectEngine::create_pinhole(float field_of_view, int width, int height)
{
    pinhole_t pinhole;

    pinhole.px = (float)width / 2.f;
    pinhole.py = (float)height / 2.f;
    pinhole.fx = 605.287;
    pinhole.fy = 605.172;
    pinhole.width = width;
    pinhole.height = height;
    return pinhole;
}

void AzureKinectEngine::create_undistortion_lut(const k4a_calibration_t *calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t *pinhole,
                                    k4a_image_t lut)
{
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    k4a_float3_t ray;
    ray.xyz.z = 1.f;

    int src_width = calibration->depth_camera_calibration.resolution_width;
    int src_height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        src_width = calibration->color_camera_calibration.resolution_width;
        src_height = calibration->color_camera_calibration.resolution_height;
    }

    for (int y = 0, idx = 0; y < pinhole->height; y++)
    {
        ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

        for (int x = 0; x < pinhole->width; x++, idx++)
        {
            ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

            k4a_float2_t distorted;
            int valid;
            k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

            coordinate_t src;
            // Remapping via nearest neighbor interpolation
            src.x = (int)floorf(distorted.xy.x + 0.5f);
            src.y = (int)floorf(distorted.xy.y + 0.5f);

            if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
            {
                lut_data[idx] = src;
            }
            else
            {
                lut_data[idx].x = INVALID;
                lut_data[idx].y = INVALID;
            }
        }
    }
}

void AzureKinectEngine::remapcolor(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    auto *src_data = k4a_image_get_buffer(src);
    auto *dst_data = k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint8_t) * 4);

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            dst_data[i*4+0] = (uint8_t)src_data[(lut_data[i].y * src_width + lut_data[i].x)*4+2];
            dst_data[i*4+1] = (uint8_t)src_data[(lut_data[i].y * src_width + lut_data[i].x)*4+1];
            dst_data[i*4+2] = (uint8_t)src_data[(lut_data[i].y * src_width + lut_data[i].x)*4+0];
            dst_data[i*4+3] = (uint8_t)src_data[(lut_data[i].y * src_width + lut_data[i].x)*4+3];
        }
    }
}

void AzureKinectEngine::remapdepth(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t *src_data = (uint16_t *)(void *)k4a_image_get_buffer(src);
    uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
        }
    }
}

void AzureKinectEngine::collectData()
{
    while(device)
    {
        if(!device.get_capture(&capture, std::chrono::milliseconds(1000)))
        {
            std::cout << "Timeout waiting for a capture" << std::endl;
            return;
        }

        //get imu
        k4a_imu_sample_t imu;
        bool remain_imu = device.get_imu_sample(&imu, std::chrono::milliseconds(0));
        while(remain_imu){
            //The gyroscope and accelerometer to depth.
            k4a_float3_t acc2depth;
            k4a_float3_t gyro2depth;

            acc2depth.xyz.x = a2d.rotation[0] * imu.acc_sample.xyz.x + a2d.rotation[1] * imu.acc_sample.xyz.y + a2d.rotation[2] * imu.acc_sample.xyz.z;
            acc2depth.xyz.y = a2d.rotation[3] * imu.acc_sample.xyz.x + a2d.rotation[4] * imu.acc_sample.xyz.y + a2d.rotation[5] * imu.acc_sample.xyz.z;
            acc2depth.xyz.z = a2d.rotation[6] * imu.acc_sample.xyz.x + a2d.rotation[7] * imu.acc_sample.xyz.y + a2d.rotation[8] * imu.acc_sample.xyz.z;

            gyro2depth.xyz.x = g2d.rotation[0] * imu.gyro_sample.xyz.x + g2d.rotation[1] * imu.gyro_sample.xyz.y + g2d.rotation[2] * imu.gyro_sample.xyz.z;
            gyro2depth.xyz.y = g2d.rotation[3] * imu.gyro_sample.xyz.x + g2d.rotation[4] * imu.gyro_sample.xyz.y + g2d.rotation[5] * imu.gyro_sample.xyz.z;
            gyro2depth.xyz.z = g2d.rotation[6] * imu.gyro_sample.xyz.x + g2d.rotation[7] * imu.gyro_sample.xyz.y + g2d.rotation[8] * imu.gyro_sample.xyz.z;

            DataReader::IMUData imu_temp(gyro2depth.xyz.x,gyro2depth.xyz.y,gyro2depth.xyz.z,
                                         acc2depth.xyz.x,acc2depth.xyz.y,acc2depth.xyz.z,imu.gyro_timestamp_usec);
            related_imu.push(imu_temp);
            for(int i=0; i<7; i++){
                remain_imu = device.get_imu_sample(&imu, std::chrono::milliseconds(0));
            }
        }

        //get image
        k4a::image color = capture.get_color_image();
        k4a::image depth = capture.get_depth_image();

        imgtime = color.get_device_timestamp().count();
        if(last_image_time < 1e-3) last_image_time = imgtime;

        //
        k4a::image transformed_depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                                          1280,
                                                          720,
                                                          1280*(int)sizeof(uint16_t));
        transdepth2color.depth_image_to_color_camera(depth, &transformed_depth);

        //thread get image
        {
            std::lock_guard<std::mutex> lock(collectlock);
            //undistort & resize
            remapcolor(color.handle(), lut, undistortedcolor);
            remapdepth(transformed_depth.handle(), lut, undistortedepth);
            newdata = true;
        }


        //reset
        color.reset(); depth.reset();
        capture.reset();
    }

}

AzureKinectEngine::AzureKinectEngine(const char *calibFilename, bool alignColourWithDepth,
                                 Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
        : BaseImageSourceEngine(calibFilename)
{
    //get device info
    auto count = k4a::device::get_installed_count();
    if(count == 0) return;
    device = k4a::device::open(K4A_DEVICE_DEFAULT);
    auto serial_num = device.get_serialnum();
    std::cout << serial_num << std::endl;

    //-----------config sensor-----------
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture
//    device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, 15625);
//change exposure tiem: 488(500) 977(1250) 1953(2500) 3906(10000) 7813(20000) 15625(30000)
    //-----------output sensor parameter-----------
    k4a_calibration_t calibration = device.get_calibration(config.depth_mode, config.color_resolution);
    g2d = calibration.extrinsics[K4A_CALIBRATION_TYPE_GYRO][K4A_CALIBRATION_TYPE_DEPTH];
    a2d = calibration.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH];

    //----------------start device----------------------
    device.start_cameras(&config);
    device.start_imu();

    transdepth2color = k4a::transformation(calibration);
    capture = k4a::capture::create();

    // Generate a pinhole model with 90 degree field of view and a resolution of 640*480 pixels
    pinhole_t pinhole = create_pinhole(90, 640, 480);
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     pinhole.width,
                     pinhole.height,
                     pinhole.width * (int)sizeof(coordinate_t),
                     &lut);

    create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_COLOR, &pinhole, lut);

    k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                     pinhole.width,
                     pinhole.height,
                     pinhole.width*4,
                     &undistortedcolor);
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                     pinhole.width,
                     pinhole.height,
                     pinhole.width*2,
                     &undistortedepth);


    //---------------init delay------------------------
    collect_frame_num = 0;
    last_image_time = 0.0f;
    newdata = false;

    //初始化标定参数
    this->calib.disparityCalib.SetStandard();
//    this->imageSize_rgb =  Vector2i(calibration.color_camera_calibration.resolution_width, calibration.color_camera_calibration.resolution_height);
    this->imageSize_rgb =  Vector2i(640, 480);//resize
    this->imageSize_d = this->imageSize_rgb;

    inputRGBImage1 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRGBImage2 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRGBImage3 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRawDepthImage1 = new ITMShortImage(this->imageSize_d, true, true);
    inputRawDepthImage2 = new ITMShortImage(this->imageSize_d, true, true);
    inputRawDepthImage3 = new ITMShortImage(this->imageSize_d, true, true);

    thread cd(&AzureKinectEngine::collectData, this);
    collectdata.swap(cd);
}

AzureKinectEngine::~AzureKinectEngine()
{
    if (device)
    {
        transdepth2color.destroy();
        device.stop_imu();
        device.stop_cameras();
        device.close();
    }
    k4a_image_release(lut);
    k4a_image_release(undistortedcolor);
    k4a_image_release(undistortedepth);

    delete inputRGBImage1;
    delete inputRGBImage2;
    delete inputRGBImage3;
    delete inputRawDepthImage1;
    delete inputRawDepthImage2;
    delete inputRawDepthImage3;
}


void AzureKinectEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
    while(!newdata)
        usleep(100);

    collect_frame_num++;

    TICK("1-readimage");
    //execute image delay
    inputRGBImage1->SetFrom(inputRGBImage2, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    inputRGBImage2->SetFrom(inputRGBImage3, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    inputRawDepthImage1->SetFrom(inputRawDepthImage2, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
    inputRawDepthImage2->SetFrom(inputRawDepthImage3, ORUtils::MemoryBlock<short>::CPU_TO_CPU);

    //collect data
    Vector4u *rgb = inputRGBImage3->GetData(MEMORYDEVICE_CPU);
    short *rawDepth = inputRawDepthImage3->GetData(MEMORYDEVICE_CPU);
    inputRGBImage3->Clear(); inputRawDepthImage3->Clear();
    {
        std::lock_guard<std::mutex> lock(collectlock);
        newdata = false;

        constexpr size_t rgb_pixel_size = sizeof(Vector4u);
        static_assert(4 == rgb_pixel_size, "sizeof(rgb pixel) must equal 4");
//    const Vector4u * color_frame = reinterpret_cast<const Vector4u*>(color.get_buffer());
        const Vector4u * color_frame = reinterpret_cast<const Vector4u*>(k4a_image_get_buffer(undistortedcolor));//resize

        constexpr size_t depth_pixel_size = sizeof(uint16_t);
        static_assert(2 == depth_pixel_size, "sizeof(depth pixel) must equal 2");
//    auto depth_frame = reinterpret_cast<const uint16_t *>(transformed_depth.get_buffer());
        auto depth_frame = reinterpret_cast<const uint16_t *>(k4a_image_get_buffer(undistortedepth));//resize

        // Let's just memcpy the data instead of using loops
        ::memcpy(rgb, color_frame, rgb_pixel_size * rgbImage->noDims.x*rgbImage->noDims.y);
        ::memcpy(rawDepth, depth_frame, depth_pixel_size * rawDepthImage->noDims.x * rawDepthImage->noDims.y);
    }

    //real input data to tracking.
    if(collect_frame_num >= 3)
    {
        rgbImage->SetFrom(inputRGBImage1, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        rawDepthImage->SetFrom(inputRawDepthImage1, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
    } else if(collect_frame_num == 2)
    {
        rgbImage->SetFrom(inputRGBImage2, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        rawDepthImage->SetFrom(inputRawDepthImage2, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
    } else if(collect_frame_num == 1)
    {
        rgbImage->SetFrom(inputRGBImage3, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        rawDepthImage->SetFrom(inputRawDepthImage3, ORUtils::MemoryBlock<short>::CPU_TO_CPU);

        cout<< "first" << related_imu.size() <<endl;
        while(related_imu.front()._t <= imgtime)    related_imu.pop();
    }

    cv::Mat gray = cv::Mat::zeros(this->imageSize_rgb.height,this->imageSize_rgb.width,CV_8UC1);
    for(int i=0;i<gray.rows;i++){
        int id = i*gray.cols;
        for(int j=0;j<gray.cols;j++) {
            int id1 = id + j;
            gray.at<uchar>(i, j) = rgb[id1].x * 0.114 + rgb[id1].y * 0.587 + rgb[id1].z * 0.2989;
        }
    }
    gray.copyTo(grayimg);

    TOCK("1-readimage");
}

bool AzureKinectEngine::hasMoreImages(void) const { return (device); }
Vector2i AzureKinectEngine::getDepthImageSize(void) const { return (device)?imageSize_d:Vector2i(0,0); }
Vector2i AzureKinectEngine::getRGBImageSize(void) const { return (device)?imageSize_rgb:Vector2i(0,0); }

void AzureKinectEngine::getRelatedIMU(vector<DataReader::IMUData> &relatedIMU)
{

    if(related_imu.front()._t <= last_image_time)
        related_imu.pop();
    cout<< "related" << related_imu.size()<<endl;

    while(!related_imu.empty())
    {
        if(related_imu.front()._t <= imgtime)
        {
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

AzureKinectEngine::AzureKinectEngine(const char *calibFilename, bool alignColourWithDepth,
                                 Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
: BaseImageSourceEngine(calibFilename)
{
	printf("compiled without RealSense Windows support\n");
}
AzureKinectEngine::~AzureKinectEngine()
{}
void AzureKinectEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool AzureKinectEngine::hasMoreImages(void) const
{ return false; }
Vector2i AzureKinectEngine::getDepthImageSize(void) const
{ return Vector2i(0,0); }
Vector2i AzureKinectEngine::getRGBImageSize(void) const
{ return Vector2i(0,0); }
void AzureKinectEngine::getRelatedIMU(vector<DataReader::IMUData> &relatedIMU)
{}
#endif

