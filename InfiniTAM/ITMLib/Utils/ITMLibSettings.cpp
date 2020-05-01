// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"
using namespace ITMLib;

#include <climits>
#include <cmath>

//use which sensor
//#define D435I
//#define ZR300
#define AZUREKINECT
//#define BADSLAM

ITMLibSettings::ITMLibSettings(void)
:	sceneParams(0.04f, 100, 0.01f, 0.1f, 6.0f, false)
{
    useIMU = false;
    //sensor parameters
#ifdef D435I
    rovio_filter_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/D435i/rovio.info";
    rovio_camera_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/D435i/realsense.yaml";
#else
#ifdef ZR300
    rovio_filter_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/realsense/rovio.info";
    rovio_camera_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/realsense/realsense.yaml";
#else
#ifdef AZUREKINECT
    rovio_filter_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/Azurekinect/rovio.info";
    rovio_camera_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/Azurekinect/realsense.yaml";
#else
#ifdef BADSLAM
    rovio_filter_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/ETH3D/rovio.info";
    rovio_camera_config = "/home/zhuzunjie/Projects/InfiniTAM/InfiniTAM/Files/ETH3D/realsense.yaml";
#endif
#endif
#endif
#endif

    //save images sequence results for side by side compare
    shotImageDir = "/home/zhuzunjie/Videos/azurekinect/homeslow/";

    //extra
    saveRefinedepth = false;
    saveTraj = false;


	// skips every other point when using the colour renderer for creating a point cloud
	skipPoints = true;

	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	createMeshingEngine = true;

#ifndef COMPILE_WITHOUT_CUDA
	deviceType = DEVICE_CUDA;
#else
#ifdef COMPILE_WITH_METAL
	deviceType = DEVICE_METAL;
#else
	deviceType = DEVICE_CPU;
#endif
#endif

	/// how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version
	swappingMode = SWAPPINGMODE_DISABLED;

	/// enables or disables approximate raycast TODO usage?
	useApproximateRaycast = false;

	/// enable or disable bilateral depth filtering
    useBilateralFilter = false;

	/// what to do on tracker failure: ignore, relocalise or stop integration - not supported in loop closure version
//	behaviourOnFailure = FAILUREMODE_RELOCALISE; //TODO not complete
	behaviourOnFailure = FAILUREMODE_IGNORE;

	/// switch between various library modes - basic, with loop closure, etc.
	libMode = LIBMODE_BASIC;
//    libMode = LIBMODE_LOOPCLOSURE;

    //parameters for loop closure detection
    k_LoopCloseNeighbours = 5;
    F_MaxDistatTemptReloc = 0.10f; //good for rent1/slow
    F_MinDistAddKeyframe = 0.10f;// rgbd
    separateThreadGlobalAdjustment = false;
    numFerns = 1000;
    numDecisionsPerFern = 4;
    relocType = FernRelocLib::RelocType::Both;


    // FastFusion
    trackerConfig = "type=fastfusion,levels=rrrtb,minstep=1e-5,"
                    "outlierC=0.25,outlierF=0.15,"
                    "numiterC=10,numiterF=30,failureDec=30.0"; // 5 for normal, 20 for loop closure
//    useIMU = true;

/*    //Colour only tracking, using rendered colours
//	trackerConfig = "type=rgb,levels=rrbb";

    // Default ICP tracking
//	trackerConfig = "type=icp,levels=rrrbb,minstep=1e-4,"
//					"outlierC=0.01,outlierF=0.002,"
//					"numiterC=20,numiterF=100,failureDec=30.0"; // 5 for normal, 20 for loop closure

    //	 //Depth-only extended tracker:
//	trackerConfig = "type=extended,levels=rrbb,useDepth=1,minstep=1e-4,"
//					  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
//					  "numiterC=20,numiterF=50,tukeyCutOff=8,"
//					  "framesToSkip=20,framesToWeight=50,failureDec=20.0";

	// For hybrid intensity+depth tracking:
//	trackerConfig = "type=extended,levels=bbb,useDepth=1,useColour=1,"
//					  "colourWeight=0.3,minstep=1e-4,"
//					  "outlierColourC=0.175,outlierColourF=0.005,"
//					  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
//					  "numiterC=20,numiterF=50,tukeyCutOff=8,"
//					  "framesToSkip=20,framesToWeight=50,failureDec=30.0";

	//trackerConfig = "type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,outlierF=0.005,numiterC=4,numiterF=2";
	//trackerConfig = "type=extendedimu,levels=ttb,minstep=5e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=5,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";*/
}

MemoryDeviceType ITMLibSettings::GetMemoryType() const
{
	return deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
}
