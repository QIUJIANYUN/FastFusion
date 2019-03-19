//
// Created by zhuzunjie on 19-2-26.
//

#pragma once

#include "ITMTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Tracking/ITMImageHierarchy.h"
#include "../../Objects/Tracking/ITMTemplatedHierarchyLevel.h"
#include "../../Objects/Tracking/ITMSceneHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

#include "../../../ORUtils/HomkerMap.h"
#include "../../../ORUtils/SVMClassifier.h"

//#include "../../../ROVIO/RovioTracker.h"

namespace ITMLib
{
    // Base class for engine performing robust fast motion tracking.
    class FastFusionTracker : public ITMTracker
    {
    private:
//        RovioTracker *roviotracker;

        const ITMLowLevelEngine *lowLevelEngine;
        ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
        ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy;

        ITMTrackingState *trackingState; const ITMView *view;

        int *noIterationsPerLevel;

        float terminationThreshold;

        void PrepareForEvaluation();
        void SetEvaluationParams(int levelId);

        void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
        void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
        bool HasConverged(float *step) const;

        void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);

        void UpdatePoseQuality(int noValidPoints_old, float *hessian_good, float f_old);

//        void InitPoseWithROVIO();

        ORUtils::HomkerMap *map;
        ORUtils::SVMClassifier *svmClassifier;
        Vector4f mu, sigma;
    protected:
        float *distThresh;

        int levelId;
        TrackerIterationType iterationType;

        Matrix4f scenePose;
        ITMSceneHierarchyLevel *sceneHierarchyLevel;
        ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;

        virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

    public:
        void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

        bool requiresColourRendering() const { return false; }
        bool requiresDepthReliability() const { return false; }
        bool requiresPointCloudRendering() const { return true; }

        void SetupLevels(int numIterCoarse, int numIterFine, float distThreshCoarse, float distThreshFine);

        FastFusionTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                        float terminationThreshold, float failureDetectorThreshold,
                        const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
        virtual ~FastFusionTracker(void);
    };
}