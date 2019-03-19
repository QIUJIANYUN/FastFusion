// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CUDA.h"

#include "../Shared/ITMViewBuilder_Shared.h"
#include "../../../../ORUtils/CUDADefines.h"
#include "../../../../ORUtils/MemoryBlock.h"

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CUDA::ITMViewBuilder_CUDA(const ITMRGBDCalib& calib):ITMViewBuilder(calib) { }
ITMViewBuilder_CUDA::~ITMViewBuilder_CUDA(void) { }

//---------------------------------------------------------------------------
//
// kernel function declaration 
//
//---------------------------------------------------------------------------

__global__ void alignDepth2Color_device(float *d_out, const float *d_in, const Matrix4f d2c, const Vector4f depth,
                                        const Vector4f color, Vector2i imgSize);
__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize);
__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams);
__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void zr300_depth_denoise_piece_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void zr300_depth_denoise_point_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void filterBilateral_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void ComputeNormalAndWeight_device(const float* depth_in, Vector4f* normal_out, float *sigmaL_out, Vector2i imgDims, Vector4f intrinsic);

//---------------------------------------------------------------------------
//
// host methods
//
//---------------------------------------------------------------------------

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, true);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, true);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(rawDepthImage->noDims, true, true);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, true);
		}
	}

	ITMView *view = *view_ptr;

	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev = new ITMUChar4Image(rgbImage->noDims, true, true);
		else view->rgb_prev->SetFrom(view->rgb, MemoryBlock<Vector4u>::CUDA_TO_CUDA);
	}	

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CUDA);

	switch (view->calib.disparityCalib.GetType())
	{
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib.intrinsics_d), view->calib.disparityCalib.GetParams());
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib.disparityCalib.GetParams());
		break;
	default:
		break;
	}

	if (useBilateralFilter)
	{
		//3 step filtering
		//1. 双边滤波
		this->DepthFiltering( this->floatImage,view->depth);
		//2. 通过距离内有效点的数量滤波
		this->zr300_depth_denoise_point(view->depth, this->floatImage);
		//3. 去除噪点（片）
		this->zr300_depth_denoise_piece(this->floatImage, view->depth);
		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CUDA_TO_CUDA);
		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CUDA_TO_CPU);
	}

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, view->depth, view->calib.intrinsics_d.projectionParamsSimple.all);
	}
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == NULL) 
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, true);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, true);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(depthImage->noDims, true, true);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(depthImage->noDims, true, true);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(depthImage->noDims, true, true);
		}
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, cv::Mat *grayimg, std::vector<DataReader::IMUData> *relatedIMU, double imgtime, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, true);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, true);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(rawDepthImage->noDims, true, true);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, true);
		}
	}

	ITMView *view = *view_ptr;

	view->grayimg = grayimg;
	view->imgtime = imgtime;
	view->relatedIMU = relatedIMU;

	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev = new ITMUChar4Image(rgbImage->noDims, true, true);
		else view->rgb_prev->SetFrom(view->rgb, MemoryBlock<Vector4u>::CUDA_TO_CUDA);
	}

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CUDA);

	switch (view->calib.disparityCalib.GetType())
	{
		case ITMDisparityCalib::TRAFO_KINECT:
			this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib.intrinsics_d), view->calib.disparityCalib.GetParams());
			break;
		case ITMDisparityCalib::TRAFO_AFFINE:
			this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib.disparityCalib.GetParams());
			break;
		default:
			break;
	}

	if (useBilateralFilter)
	{
		//3 step filtering
		//1. 双边滤波
		this->DepthFiltering( this->floatImage,view->depth);
		//2. 通过距离内有效点的数量滤波
		this->zr300_depth_denoise_point(view->depth, this->floatImage);
		//3. 去除噪点（片）
		this->zr300_depth_denoise_piece(this->floatImage, view->depth);
		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CUDA_TO_CUDA);
		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CUDA_TO_CPU);
	}

    if (view->aligned_depth != NULL) delete view->aligned_depth;
    view->aligned_depth = new ITMFloatImage(rawDepthImage->noDims, true, true);
	this->depth_Align2_color(view->aligned_depth, view->depth, &(view->calib.trafo_depth_to_rgb), &(view->calib.intrinsics_d), &(view->calib.intrinsics_rgb));
	view->aligned_depth->SetFrom(view->aligned_depth, MemoryBlock<float>::CUDA_TO_CPU);

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, view->depth, view->calib.intrinsics_d.projectionParamsSimple.all);
	}
}

void ITMViewBuilder_CUDA::depth_Align2_color(ITMFloatImage *depth_out, const ITMFloatImage *depth_in, const ITMExtrinsics *depthExtrinsics, const ITMIntrinsics *depthIntrinsics, const ITMIntrinsics *colorIntrinsics)
{
	Vector2i imgSize = depth_in->noDims;

	const float *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	alignDepth2Color_device<< <gridSize, blockSize >> >(d_out, d_in, depthExtrinsics->calib, depthIntrinsics->projectionParamsSimple.all, colorIntrinsics->projectionParamsSimple.all, imgSize);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDisparityToDepth_device << <gridSize, blockSize >> >(d_out, d_in, disparityCalibParams, fx_depth, imgSize);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDepthAffineToFloat_device << <gridSize, blockSize >> >(d_out, d_in, imgSize, depthCalibParams);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgDims = image_in->noDims;

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	filterDepth_device << <gridSize, blockSize >> >(imageData_out, imageData_in, imgDims);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::zr300_depth_denoise_piece(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgDims = image_in->noDims;

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	zr300_depth_denoise_piece_device << <gridSize, blockSize >> >(imageData_out, imageData_in, imgDims);
}

void ITMViewBuilder_CUDA::zr300_depth_denoise_point(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgDims = image_in->noDims;

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	zr300_depth_denoise_point_device << <gridSize, blockSize >> >(imageData_out, imageData_in, imgDims);
}

void ITMViewBuilder_CUDA::BilateralFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgDims = image_in->noDims;

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	filterBilateral_device << <gridSize, blockSize >> >(imageData_out, imageData_in, imgDims);
}

void ITMViewBuilder_CUDA::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CUDA);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	ComputeNormalAndWeight_device << <gridSize, blockSize >> >(depthData_in, normalData_out, sigmaZData_out, imgDims, intrinsic);
	ORcudaKernelCheck;
}

//---------------------------------------------------------------------------
//
// kernel function implementation
//
//---------------------------------------------------------------------------

__global__ void alignDepth2Color_device(float *d_out, const float *d_in, const Matrix4f d2c, const Vector4f depth,
                                        const Vector4f color, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	int locId = x + y * imgSize.x;
	float d = d_in[locId];
    if(d<1e-3) return;

	float d1,d2,d3;
    d1 = (x - depth.z)*d/depth.x;
    d2 = (y - depth.w)*d/depth.y;
    d3 = d;

	float c1,c2,c3;
    c1 = d2c.m00*d1 + d2c.m10*d2 + d2c.m20*d3 + d2c.m30;
    c2 = d2c.m01*d1 + d2c.m11*d2 + d2c.m21*d3 + d2c.m31;
    c3 = d2c.m02*d1 + d2c.m12*d2 + d2c.m22*d3 + d2c.m32;

	int u = (int)(color.x * c1/c3 + color.z);
	int v = (int)(color.y * c2/c3 + color.w);

	if(u>=0 && u<imgSize.x && v>=0 && v<imgSize.y)
		d_out[u + v * imgSize.x] = d;
}
__global__ void convertDisparityToDepth_device(float *d_out, const short *d_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 6 || x > imgDims.x - 6 || y < 6 || y > imgDims.y - 6) return;

	filterDepth(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void zr300_depth_denoise_piece_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2) return;

	depth_denoise_piece(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void zr300_depth_denoise_point_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 3 || x > imgDims.x - 3 || y < 3 || y > imgDims.y - 3) return;

	depth_denoise_point(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void filterBilateral_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 10 || x > imgDims.x - 10 || y < 10 || y > imgDims.y - 10) return;

	filterBilateral(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void ComputeNormalAndWeight_device(const float* depth_in, Vector4f* normal_out, float *sigmaZ_out, Vector2i imgDims, Vector4f intrinsic)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	int idx = x + y * imgDims.x;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}
	else
	{
		computeNormalAndWeight(depth_in, normal_out, sigmaZ_out, x, y, imgDims, intrinsic);
	}
}

