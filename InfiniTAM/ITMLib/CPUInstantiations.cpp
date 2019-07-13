// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Core/ITMBasicEngine.tpp"
#include "Core/ITMMultiEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.tpp"
#include "Engines/MultiScene/ITMMapGraphManager.tpp"
#include "Engines/Visualisation/CPU/ITMMultiVisualisationEngine_CPU.tpp"
#include "Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp"
#include "Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "Trackers/ITMTrackerFactory.h"

namespace ITMLib
{
	template class ITMBasicEngine<ITMVoxel, ITMVoxelIndex>;
	template class ITMMultiEngine<ITMVoxel, ITMVoxelIndex>;
	template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
	template class ITMVoxelMapGraphManager<ITMVoxel, ITMVoxelIndex>;
	template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMMultiMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
}
