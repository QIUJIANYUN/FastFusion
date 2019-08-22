// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "RealSenseEngine.h"

#include <cstdio>
#include <stdexcept>
#include <iomanip>
#include <iostream>

#include "../ORUtils/FileUtils.h"

//static vector<DataReader::IMUData> related_imu;
static queue<DataReader::IMUData> related_imu;
#ifdef COMPILE_WITH_RealSense

static bool default_depth2color = false; // 这里估计也是计算出来的,而且用的cpu,需要12ms

using namespace InputSource;
using namespace ITMLib;

#define tt 1000.0

class RealSenseEngine::PrivateData
{
	public:
	PrivateData(void) : dev(NULL){}
	rs::device *dev = NULL;
	rs::context ctx;
};

RealSenseEngine::RealSenseEngine(const char *calibFilename, bool alignColourWithDepth,
                                 Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
: BaseImageSourceEngine(calibFilename),
  colourStream(alignColourWithDepth ? rs::stream::color_aligned_to_depth : rs::stream::color)
{
    //init delay
    collect_frame_num = 0;

    //初始化标定参数
	this->calib.disparityCalib.SetStandard();
//	this->calib.trafo_rgb_to_depth = ITMExtrinsics();
	this->imageSize_d = requested_imageSize_d;
	this->imageSize_rgb = requested_imageSize_rgb;

    inputRGBImage1 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRGBImage2 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRGBImage3 = new ITMUChar4Image(this->imageSize_rgb, true, true);
    inputRawDepthImage1 = new ITMShortImage(this->imageSize_d, true, true);
    inputRawDepthImage2 = new ITMShortImage(this->imageSize_d, true, true);
    inputRawDepthImage3 = new ITMShortImage(this->imageSize_d, true, true);


	data = new RealSenseEngine::PrivateData();
	printf("There are %d connected RealSense devices.\n", data->ctx.get_device_count());
	if (data->ctx.get_device_count() == 0) {
		dataAvailable = false;
		delete data;
		data = NULL;
		return;
	}

	data->dev = data->ctx.get_device(0);

	data->dev->enable_stream(rs::stream::depth, imageSize_d.x, imageSize_d.y, rs::format::z16, 30);
	data->dev->enable_stream(rs::stream::color, imageSize_rgb.x, imageSize_rgb.y, rs::format::rgb8, 30);

    static double gtime=0.0,atime=0.0,accel1,accel2,accel3;
    static DataReader::IMUData imu(0,0,0,0,0,0,0);
    auto motion_callback = [](rs::motion_data entry)
    {
        if (entry.timestamp_data.source_id == RS_EVENT_IMU_ACCEL)
        {
            imu._a[0] = entry.axes[0]; imu._a[1] = entry.axes[1]; imu._a[2] = entry.axes[2];

            double t = entry.timestamp_data.timestamp *tt;
            if(t < gtime || gtime == 0.0 || atime >= gtime )
            {
                atime = t;
                accel1 = entry.axes[0];
                accel2 = entry.axes[1];
                accel3 = entry.axes[2];
            }
            else
            {
                imu._a[0] = ((t-gtime)*accel1+(gtime-atime)*entry.axes[0])/(t-atime);
                imu._a[1] = ((t-gtime)*accel2+(gtime-atime)*entry.axes[1])/(t-atime);
                imu._a[2] = ((t-gtime)*accel3+(gtime-atime)*entry.axes[2])/(t-atime);
                related_imu.push(imu);

                atime = t;
                accel1 = entry.axes[0];
                accel2 = entry.axes[1];
                accel3 = entry.axes[2];
            }
        }

        if (entry.timestamp_data.source_id == RS_EVENT_IMU_GYRO)
        {
            imu._g[0] = entry.axes[0];
            imu._g[1] = entry.axes[1];
            imu._g[2] = entry.axes[2];

            gtime = entry.timestamp_data.timestamp*tt;
            imu._t = gtime;
        }
    };

    // ... and the timestamp packets (DS4.1/FishEye Frame, GPIOS...)
    auto timestamp_callback = [](rs::timestamp_data entry)
    {

    };
    if (data->dev->supports(rs::capabilities::motion_events))
    {
        data->dev->enable_motion_tracking(motion_callback, timestamp_callback);
    }

	rs::intrinsics intrinsics_depth = data->dev->get_stream_intrinsics(rs::stream::depth);
	rs::intrinsics intrinsics_rgb = data->dev->get_stream_intrinsics(colourStream);

	this->calib.intrinsics_rgb.SetFrom(this->imageSize_rgb[0], this->imageSize_rgb[1], intrinsics_rgb.fx, intrinsics_rgb.fy, intrinsics_rgb.ppx, intrinsics_rgb.ppy);
    this->calib.intrinsics_d.SetFrom(this->imageSize_d[0], this->imageSize_d[1], intrinsics_depth.fx, intrinsics_depth.fy, intrinsics_depth.ppx, intrinsics_depth.ppy);

    if(default_depth2color)
    {
        rs::intrinsics intrinsics_aligneddepth = data->dev->get_stream_intrinsics(rs::stream::depth_aligned_to_color);
        this->calib.intrinsics_d.SetFrom(this->imageSize_d[0], this->imageSize_d[1], intrinsics_aligneddepth.fx, intrinsics_aligneddepth.fy, intrinsics_aligneddepth.ppx, intrinsics_aligneddepth.ppy);
    }

    Matrix4f extrinsics_c2d, extrinsics_d2c;
/*    rs::extrinsics rs_extrinsics_c2d = data->dev->get_extrinsics(colourStream, rs::stream::depth);
	rs::extrinsics rs_extrinsics_d2c = data->dev->get_extrinsics(rs::stream::depth, colourStream);
	extrinsics_c2d.m00 = rs_extrinsics_c2d.rotation[0]; extrinsics_c2d.m10 = rs_extrinsics_c2d.rotation[1]; extrinsics_c2d.m20 = rs_extrinsics_c2d.rotation[2];
	extrinsics_c2d.m01 = rs_extrinsics_c2d.rotation[3]; extrinsics_c2d.m11 = rs_extrinsics_c2d.rotation[4]; extrinsics_c2d.m21 = rs_extrinsics_c2d.rotation[5];
	extrinsics_c2d.m02 = rs_extrinsics_c2d.rotation[6]; extrinsics_c2d.m12 = rs_extrinsics_c2d.rotation[7]; extrinsics_c2d.m22 = rs_extrinsics_c2d.rotation[8];
    extrinsics_c2d.m30 = rs_extrinsics_c2d.translation[0];
    extrinsics_c2d.m31 = rs_extrinsics_c2d.translation[1];
    extrinsics_c2d.m32 = rs_extrinsics_c2d.translation[2];
	extrinsics_c2d.m03 = 0.0f; extrinsics_c2d.m13 = 0.0f; extrinsics_c2d.m23 = 0.0f; extrinsics_c2d.m33 = 1.0f;

    extrinsics_d2c.m00 = rs_extrinsics_d2c.rotation[0]; extrinsics_d2c.m10 = rs_extrinsics_d2c.rotation[1]; extrinsics_d2c.m20 = rs_extrinsics_d2c.rotation[2];
    extrinsics_d2c.m01 = rs_extrinsics_d2c.rotation[3]; extrinsics_d2c.m11 = rs_extrinsics_d2c.rotation[4]; extrinsics_d2c.m21 = rs_extrinsics_d2c.rotation[5];
    extrinsics_d2c.m02 = rs_extrinsics_d2c.rotation[6]; extrinsics_d2c.m12 = rs_extrinsics_d2c.rotation[7]; extrinsics_d2c.m22 = rs_extrinsics_d2c.rotation[8];
    extrinsics_d2c.m30 = rs_extrinsics_d2c.translation[0];
    extrinsics_d2c.m31 = rs_extrinsics_d2c.translation[1];
    extrinsics_d2c.m32 = rs_extrinsics_d2c.translation[2];
    extrinsics_d2c.m03 = 0.0f; extrinsics_d2c.m13 = 0.0f; extrinsics_d2c.m23 = 0.0f; extrinsics_d2c.m33 = 1.0f;
    cout << extrinsics_c2d << endl;
    cout << extrinsics_d2c << endl;*/

    if(default_depth2color)
    {
        extrinsics_c2d.setIdentity(); extrinsics_d2c.setIdentity();
        this->calib.trafo_rgb_to_depth.SetFrom(extrinsics_c2d); this->calib.trafo_depth_to_rgb.SetFrom(extrinsics_d2c);
    }



	this->calib.disparityCalib.SetFrom(data->dev->get_depth_scale(), 0.0f,
		ITMDisparityCalib::TRAFO_AFFINE);

	data->dev->start(rs::source::all_sources);
}

RealSenseEngine::~RealSenseEngine()
{
	if (data != NULL)
	{
		data->dev->stop(rs::source::all_sources);
        data->dev->disable_motion_tracking();
		delete data;
	}
    delete inputRGBImage1;
	delete inputRGBImage2;
	delete inputRGBImage3;
    delete inputRawDepthImage1;
    delete inputRawDepthImage2;
    delete inputRawDepthImage3;
}


void RealSenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	dataAvailable = false;

	last_image_time = data->dev->get_frame_timestamp(rs::stream::depth) * tt;
//	cout << "last: " << last_image_time << endl;
	// get frames
	data->dev->wait_for_frames();

    double Time1 = (double)cvGetTickCount();
    collect_frame_num++;
    imgtime = data->dev->get_frame_timestamp(rs::stream::depth) *tt;

    inputRGBImage1->SetFrom(inputRGBImage2, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    inputRGBImage2->SetFrom(inputRGBImage3, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    inputRawDepthImage1->SetFrom(inputRawDepthImage2, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
    inputRawDepthImage2->SetFrom(inputRawDepthImage3, ORUtils::MemoryBlock<short>::CPU_TO_CPU);

    double Time2 = (double)cvGetTickCount();

    const uint16_t * depth_frame; const uint8_t * color_frame;
    if(default_depth2color) depth_frame = reinterpret_cast<const uint16_t *>(data->dev->get_frame_data(rs::stream::depth_aligned_to_color));
    else                    depth_frame = reinterpret_cast<const uint16_t *>(data->dev->get_frame_data(rs::stream::depth));
    color_frame = reinterpret_cast<const uint8_t*>(data->dev->get_frame_data(colourStream));

	// setup infinitam frames
    Vector4u *rgba = inputRGBImage3->GetData(MEMORYDEVICE_CPU);
	short *rawDepth = inputRawDepthImage3->GetData(MEMORYDEVICE_CPU);

    inputRGBImage3->Clear(); inputRawDepthImage3->Clear();

    Vector2i noDims = inputRawDepthImage3->noDims;
	for (int y = 0; y < noDims.y; y++) for (int x = 0; x < noDims.x; x++) rawDepth[x + y * noDims.x] = *depth_frame++;
	for (int i = 0; i < inputRGBImage3->noDims.x * 3 * inputRGBImage3->noDims.y ; i+=3) {
		Vector4u newPix;
		newPix.x = color_frame[i]; newPix.y = color_frame[i+1]; newPix.z = color_frame[i+2];
		newPix.w = 255;
		rgba[i/3] = newPix;
	}
    double Time3 = (double)cvGetTickCount();
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
        while(related_imu.front()._t <= imgtime)
            related_imu.pop();
    }

    double Time4 = (double)cvGetTickCount();

    cv::Mat gray = cv::Mat::zeros(480,640,CV_8UC1);
    for(int i=0;i<480;i++){
        int id = i*640;
        for(int j=0;j<640;j++) {
            int id1 = 3 * (id + j);
            gray.at<uchar>(i, j) = color_frame[id1] * 0.114 + color_frame[id1+1] * 0.587 + color_frame[id1+2] * 0.2989;
        }
    }
    gray.copyTo(grayimg);
    double Time5 = (double)cvGetTickCount();
    dataAvailable = true;

    printf( "run time = %gms\n", (Time2-Time1) /(cvGetTickFrequency()*1000) );//毫秒
    printf( "run time = %gms\n", (Time3-Time2) /(cvGetTickFrequency()*1000) );//毫秒
    printf( "run time = %gms\n", (Time4-Time3) /(cvGetTickFrequency()*1000) );//毫秒
    printf( "run time = %gms\n", (Time5-Time4) /(cvGetTickFrequency()*1000) );//毫秒
    printf( "run time = %gms\n", (Time5-Time1) /(cvGetTickFrequency()*1000) );//毫秒
}

bool RealSenseEngine::hasMoreImages(void) const { return (data!=NULL); }
Vector2i RealSenseEngine::getDepthImageSize(void) const { return (data!=NULL)?imageSize_d:Vector2i(0,0); }
Vector2i RealSenseEngine::getRGBImageSize(void) const { return (data!=NULL)?imageSize_rgb:Vector2i(0,0); }

void RealSenseEngine::getRelatedIMU(vector<DataReader::IMUData> &relatedIMU)
{

    if(related_imu.front()._t <= last_image_time)
        related_imu.pop();
    cout << related_imu.size() << endl;

    while(related_imu.front()._t <= imgtime)
    {
//        cout << related_imu.front()._t << endl;
        relatedIMU.push_back(related_imu.front());
        related_imu.pop();
    }

    cout << "current: " << relatedIMU.size()<< endl;
}

#else

using namespace InputSource;

RealSenseEngine::RealSenseEngine(const char *calibFilename, bool alignColourWithDepth,
                                 Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
: BaseImageSourceEngine(calibFilename)
{
	printf("compiled without RealSense Windows support\n");
}
RealSenseEngine::~RealSenseEngine()
{}
void RealSenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool RealSenseEngine::hasMoreImages(void) const
{ return false; }
Vector2i RealSenseEngine::getDepthImageSize(void) const
{ return Vector2i(0,0); }
Vector2i RealSenseEngine::getRGBImageSize(void) const
{ return Vector2i(0,0); }
void RealSenseEngine::getRelatedIMU(vector<DataReader::IMUData> &relatedIMU)
{}
#endif

