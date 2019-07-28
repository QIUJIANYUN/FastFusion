// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "../../FernRelocLib/Relocaliser.h"

#include <string>

namespace ITMLib
{
	class ITMLibSettings
	{
	public:
		/// The device used to run the DeviceAgnostic code
		typedef enum {
			DEVICE_CPU,
			DEVICE_CUDA,
			DEVICE_METAL
		} DeviceType;

		typedef enum
		{
			FAILUREMODE_RELOCALISE,
			FAILUREMODE_IGNORE,
			FAILUREMODE_STOP_INTEGRATION
		} FailureMode;
        
		typedef enum
		{
			SWAPPINGMODE_DISABLED,
			SWAPPINGMODE_ENABLED,
			SWAPPINGMODE_DELETE
		} SwappingMode;

		typedef enum
		{
			LIBMODE_BASIC,
			LIBMODE_LOOPCLOSURE
		}LibMode;

		/// Select the type of device to use
		DeviceType deviceType;

		bool useApproximateRaycast;

		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;
        
		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;
		LibMode libMode;

		const char *trackerConfig;
		bool useIMU;

        std::string shotImageDir;
        bool saveTraj;
        std::string traj_save_dir;

        //parameters
        std::string rovio_filter_config;
        std::string rovio_camera_config;

        //---------------LC parameters------------------
        // number of nearest neighbours to find in the loop closure detection
        int k_LoopCloseNeighbours;
        // maximum distance reported by LCD library to attempt relocalisation
        float F_MaxDistatTemptReloc; //d
        // maximum distance, larger than which we must add keyframe.
        float F_MinDistAddKeyframe;// d
        // loop closure global adjustment runs on a separat thread
        bool separateThreadGlobalAdjustment;
        //Fern Number
        int numFerns;
        //Decisions per ferns
        int numDecisionsPerFern;
        FernRelocLib::RelocType relocType;

        /// Further, scene specific parameters such as voxel size
		ITMSceneParams sceneParams;

		ITMLibSettings(void);
		virtual ~ITMLibSettings(void) {}

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&);
		ITMLibSettings& operator=(const ITMLibSettings&);

		MemoryDeviceType GetMemoryType() const;
	};
}
