// Copyright Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

#include "../ORUtils/IMUdata.h"
#include <queue>

#ifdef COMPILE_WITH_RealSense2
#include "librealsense2/rs.hpp"
#include "librealsense2/rs_advanced_mode.hpp"
namespace rs2 { class pipeline; class context; class device; }
#endif

namespace InputSource {
	
	class RealSense2Engine : public BaseImageSourceEngine
	{
	private:
		bool dataAvailable;

        double last_image_time;

        int collect_frame_num;
        ITMUChar4Image *inputRGBImage1, *inputRGBImage2, *inputRGBImage3;
        ITMShortImage *inputRawDepthImage1, *inputRawDepthImage2, *inputRawDepthImage3;

        Vector2i imageSize_rgb, imageSize_d;

#ifdef COMPILE_WITH_RealSense2
		std::unique_ptr<rs2::context> ctx;
		std::unique_ptr<rs2::device> device;
		std::unique_ptr<rs2::pipeline> pipe;
#endif
		
	public:
		RealSense2Engine(const char *calibFilename, bool alignColourWithDepth = false,
						 Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));
		~RealSense2Engine();
		
		bool hasMoreImages(void) const;
		void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
		Vector2i getDepthImageSize(void) const;
		Vector2i getRGBImageSize(void) const;
        void getRelatedIMU(vector<DataReader::IMUData> &relatedIMU);
	};
	
}

