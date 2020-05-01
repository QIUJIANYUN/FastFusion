// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Objects/Misc/ITMIMUCalibrator.h"
#include "../../FernRelocLib/Relocaliser.h"

#include "../Engines/MultiScene/ITMActiveMapManager.h"
#include "../Engines/MultiScene/ITMGlobalAdjustmentEngine.h"
#include "../Engines/Visualisation/Interface/ITMMultiVisualisationEngine.h"
#include "../Engines/Meshing/ITMMultiMeshingEngineFactory.h"

#include "../../ROVIO/RovioTracker.h"

#include "opencv2/core/core.hpp"


#include <vector>

namespace ITMLib
{
	/** \brief
	*/
	template <typename TVoxel, typename TIndex>
	class ITMMultiEngine : public ITMMainEngine
	{
	private:
        ITMExtrinsics depth2imu;

		const ITMLibSettings *settings;

		ITMLowLevelEngine *lowLevelEngine;
		ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
		ITMMultiVisualisationEngine<TVoxel, TIndex> *multiVisualisationEngine;

		ITMMultiMeshingEngine<TVoxel, TIndex> *meshingEngine;

		ITMViewBuilder *viewBuilder;
		ITMTrackingController *trackingController;
		ITMTracker *tracker;

		RovioTracker *rovioTracker;

		ITMIMUCalibrator *imuCalibrator;
		ITMDenseMapper<TVoxel, TIndex> *denseMapper;

		FernRelocLib::Relocaliser<float, ORUtils::Vector4<unsigned char>> *relocaliser;

		ITMVoxelMapGraphManager<TVoxel, TIndex> *mapManager;
		ITMActiveMapManager *mActiveDataManager;
		ITMGlobalAdjustmentEngine *mGlobalAdjustmentEngine;
		bool mScheduleGlobalAdjustment;

		Vector2i trackedImageSize;
		ITMRenderState *renderState_freeview;
		ITMRenderState *renderState_multiscene;
		int freeviewLocalMapIdx;

		/// Pointer for storing the current input frame
		ITMView *view;

		std::vector<cv::Mat> kfs;

		int trackNums;
        ofstream fkeyframe;
		//----LC parameters
        // number of nearest neighbours to find in the loop closure detection
        int loopcloseneighbours;
        // maximum distance reported by LCD library to attempt relocalisation
        float maxdistattemptreloc; //d
        // maximum distance, larger than which we must add keyframe.
        float minDistAddKeyframe;// d
        // loop closure global adjustment runs on a separat thread
        bool separateThreadGlobalAdjustment;

        //Fern Number
        int numFerns;
        //Decisions per ferns
        int numDecisionsPerFern;
        FernRelocLib::RelocType relocType;


    public:
		ITMView* GetView() { return view; }

		ITMTrackingState* GetTrackingState(void);

		/// Process a frame with rgb and depth images and (optionally) a corresponding imu measurement
		ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, double imgtime, cv::Mat *grayimg = NULL, ITMIMUMeasurement *imuMeasurement = NULL, std::vector<DataReader::IMUData> *relatedIMU = NULL);

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

		void changeFreeviewLocalMapIdx(ORUtils::SE3Pose *pose, int newIdx);
		void setFreeviewLocalMapIdx(int newIdx)
		{
			freeviewLocalMapIdx = newIdx;
		}
		int getFreeviewLocalMapIdx(void) const
		{
			return freeviewLocalMapIdx;
		}
		int findPrimaryLocalMapIdx(void) const
		{
			return mActiveDataManager->findPrimaryLocalMapIdx();
		}

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveSceneToMesh(const char *fileName);

		/// save and load the full scene and relocaliser (if any) to/from file
		void SaveToFile();
		void LoadFromFile();

		//void writeFullTrajectory(void) const;
		//void SaveSceneToMesh(const char *objFileName);


		/** \brief Constructor
			Ommitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib &calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1, -1));
		~ITMMultiEngine(void);
	};
}
