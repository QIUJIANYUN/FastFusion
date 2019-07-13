// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>

#include "UIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/PicoFlexxEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/RealSense2Engine.h"
#include "../../InputSource/FFMPEGReader.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
static void CreateDefaultImageSource(ImageSourceEngine* & imageSource, const char *arg1 = NULL, const char *arg2 = NULL, const char *arg3 = NULL, const char *arg4 = NULL, const char *arg5 = NULL)
{
	const char *calibFile = arg1;
	const char *filename1 = arg2;
	const char *filename2 = arg3;
	const char *sequence = arg4;
	const char *filename_imu = arg5;

	std::fstream _file;
	_file.open(filename_imu,ios::in);
	if(!_file) filename_imu = "";

	printf("using calibration file: %s\n", calibFile);

	if ((imageSource == NULL) && (filename2 != NULL))
	{
		printf("USE file input: \n use using rgb images: %s\n using depth images: %s\n", filename1, filename2);
		if(filename_imu != NULL)
		{
			printf("using zr300 data: %s\n", filename_imu);
			imageSource = new DatasetReader(calibFile, filename1, filename2, sequence, filename_imu);
		}
	}

    if (imageSource == NULL)
	{
		printf("trying RealSense device\n");
		imageSource = new RealSenseEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

    if (imageSource == NULL)
    {
        printf("trying RealSense device\n");
        imageSource = new RealSense2Engine(calibFile);
        if (imageSource->getDepthImageSize().x == 0)
        {
            delete imageSource;
            imageSource = NULL;
        }
    }

    //    if (imageSource == NULL)
//	{
//		printf("trying MS Kinect 2 device\n");
//		imageSource = new Kinect2Engine(calibFile);
//		if (imageSource->getDepthImageSize().x == 0)
//		{
//			delete imageSource;
//			imageSource = NULL;
//		}
//	}

/*
	if ((imageSource == NULL) && (filename1 != NULL) && (filename_imu == NULL))
	{
		imageSource = new InputSource::FFMPEGReader(calibFile, filename1, filename2);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		// If no calibration file specified, use the factory default calibration
		bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

		printf("trying OpenNI device: %s - calibration: %s\n",
				filename1 ? filename1 : "<OpenNI default device>",
				useInternalCalibration ? "internal" : "from file");
		imageSource = new OpenNIEngine(calibFile, filename1, useInternalCalibration);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		printf("trying UVC device\n");
		imageSource = new LibUVCEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}



    if (imageSource == NULL)
    {
        printf("trying RealSense device with SDK 2.X (librealsense2)\n");
        imageSource = new RealSense2Engine(calibFile);
        if (imageSource->getDepthImageSize().x == 0)
        {
            delete imageSource;
            imageSource = NULL;
        }
    }



	if (imageSource == NULL)
	{
		printf("trying PMD PicoFlexx device\n");
		imageSource = new PicoFlexxEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}*/
}


int main(int argc, char** argv)
try
{
	const char *cal = "";
	const char *dir = "";
	string color_dir;
	string depth_dir;
	string sequence_dir;
	string imu_dir;
	int arg = 1;
	bool online_input = false;

	do{
		if (argv[arg] != NULL) cal = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) dir = argv[arg]; else break;
		++arg;
	} while (false);

	if(arg == 3){
		color_dir = string(dir) + "/color";
		depth_dir = string(dir) + "/depth";
		sequence_dir = string(dir) + "/COLOR.txt";
		imu_dir = string(dir) + "/IMU.txt";
	} else{
	    online_input = true;
	}

	printf("initialising ...\n");
	ImageSourceEngine *imageSource = NULL;

	if(!online_input)
	    CreateDefaultImageSource(imageSource, cal, color_dir.c_str(), depth_dir.c_str(),sequence_dir.c_str(),imu_dir.c_str());
    else
        CreateDefaultImageSource(imageSource, cal);

	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

	ITMLibSettings *internalSettings = new ITMLibSettings();

	ITMMainEngine *mainEngine = NULL;
	switch (internalSettings->libMode)
	{
	case ITMLibSettings::LIBMODE_BASIC:
		mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	case ITMLibSettings::LIBMODE_BASIC_SURFELS:
		mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	case ITMLibSettings::LIBMODE_LOOPCLOSURE:
		mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	default: 
		throw std::runtime_error("Unsupported library mode!");
		break;
	}

	UIEngine::Instance()->Initialise(argc, argv, imageSource, mainEngine, "./Files/Out", internalSettings);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

