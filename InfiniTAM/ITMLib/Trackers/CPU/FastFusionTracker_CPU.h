// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/FastFusionTracker.h"

namespace ITMLib
{
	class FastFusionTracker_CPU : public FastFusionTracker
	{
	protected:
		int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

	public:
        FastFusionTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			float terminationThreshold, float failureDetectorThreshold, const ITMLowLevelEngine *lowLevelEngine);
		~FastFusionTracker_CPU(void);
	};
}
