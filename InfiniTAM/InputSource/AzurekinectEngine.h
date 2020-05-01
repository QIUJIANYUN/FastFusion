//
// Created by zhuzunjie on 2019/8/24.
//

#ifndef FASTFUSION_V2_AZUREKINECTENGINE_H
#define FASTFUSION_V2_AZUREKINECTENGINE_H

// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifdef COMPILE_WITH_Azurekinect
#pragma comment(lib, "k4a.lib")
#include <k4a/k4a.hpp>
#endif

#include "ImageSourceEngine.h"

#include "../ORUtils/IMUdata.h"
#include <queue>
#include <thread>
#include <mutex>
#include <unistd.h>

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif
#endif

#define INVALID INT32_MIN

typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
    int x;
    int y;
} coordinate_t;


namespace InputSource {

    class AzureKinectEngine : public BaseImageSourceEngine
    {
    private:
        double last_image_time;

        int collect_frame_num;
        ITMUChar4Image *inputRGBImage1, *inputRGBImage2, *inputRGBImage3;
        ITMShortImage *inputRawDepthImage1, *inputRawDepthImage2, *inputRawDepthImage3;

        Vector2i imageSize_rgb, imageSize_d;
        queue<DataReader::IMUData> related_imu;

#ifdef COMPILE_WITH_Azurekinect
        k4a::device device;
        k4a_calibration_extrinsics_t g2d,a2d;
        k4a::capture capture;
        k4a::transformation transdepth2color;

        //
        mutex collectlock;
        thread collectdata;
        void collectData();
        bool newdata;

        // undistort and resize image.
        k4a_image_t lut;
        k4a_image_t undistortedcolor;
        k4a_image_t undistortedepth;

        static pinhole_t create_pinhole(float field_of_view, int width, int height);
        static void create_undistortion_lut(const k4a_calibration_t *calibration,
                                            const k4a_calibration_type_t camera,
                                            const pinhole_t *pinhole,
                                            k4a_image_t lut);
        static void remapcolor(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst);
        static void remapdepth(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst);
#endif

    public:
        AzureKinectEngine(const char *calibFilename, bool alignColourWithDepth = false,
                        Vector2i imageSize_rgb = Vector2i(1280, 720), Vector2i imageSize_d = Vector2i(1280, 720));
        ~AzureKinectEngine();

        bool hasMoreImages(void) const;
        void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
        Vector2i getDepthImageSize(void) const;
        Vector2i getRGBImageSize(void) const;
        void getRelatedIMU(vector<DataReader::IMUData> &relatedIMU);
    };

}


#endif //FASTFUSION_V2_AZUREKINECTENGINE_H
