// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>
#include <sys/stat.h>

#include "UIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/PicoFlexxEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/RealSense2Engine.h"
#include "../../InputSource/AzurekinectEngine.h"
#include "../../InputSource/FFMPEGReader.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

static void OnlineReader(ImageSourceEngine* & imageSource, const char *arg1 = NULL)
{
	const char *calibFile = arg1;
	printf("using calibration file: %s\n", calibFile);

    if (imageSource == NULL)
    {
        printf("trying Azurekincet device\n");
        imageSource = new AzureKinectEngine(calibFile);
        if (imageSource->getDepthImageSize().x == 0)
        {
            delete imageSource;
            imageSource = NULL;
        }
    }


    if (imageSource == NULL)
    {
        printf("trying ZR300 device\n");
        imageSource = new RealSenseEngine(calibFile);
        if (imageSource->getDepthImageSize().x == 0)
        {
            delete imageSource;
            imageSource = NULL;
        }
    }



    if (imageSource == NULL)
    {
        printf("trying D435i device\n");
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


    printf("initialising ...\n");
    ImageSourceEngine *imageSource = NULL;

    if(argc == 2){
        cal = argv[1];
        OnlineReader(imageSource, cal);
    }
    else if(argc == 4){
        //offline image loader
        string cal = string(argv[1]);   string dir = string(argv[2]);   int dm = std::atoi(argv[3]);
        imageSource = new DatasetReader(cal, dir, dm);
    }
    else{
        printf("Please input 'calibration file' for online input,\n or 'calibration file, sequence dir, dataset mode(ICL:0,TUM/BADSLAM:1,MyZR300:2,MyD435i:3,MyAzureKinect:4)' for offline input.");
        return 0;
    }

	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

	ITMLibSettings *internalSettings = new ITMLibSettings();

	if(internalSettings->saveRefinedepth){
        mkdir((string(dir) + "/filtered").c_str(),0777);
        internalSettings->depthSaveDir = string(dir) + "/filtered/";
	}


	ITMMainEngine *mainEngine = NULL;
	switch (internalSettings->libMode)
	{
	case ITMLibSettings::LIBMODE_BASIC:
		mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
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

