// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "FernConservatory.h"

#include <fstream>

using namespace FernRelocLib;

static float random_uniform01(void)
{
	return (float)rand() / (float)RAND_MAX;
}

FernConservatory::FernConservatory(int numFerns, ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> depthBounds, ORUtils::Vector2<float> colorBounds, int decisionsPerFern)
{
	mNumFerns = numFerns;
	mNumDecisions = decisionsPerFern;
	mEncoders = new FernTester[mNumFerns*decisionsPerFern];

	if(decisionsPerFern == 3)
	{
		//color only随机构造编码器
		for (int f = 0; f < mNumFerns; ++f)
		{
			int x = (int)floor(random_uniform01() * imgSize.x);
			int y = (int)floor(random_uniform01() * imgSize.y);
			for (int d = 0; d < decisionsPerFern; ++d)
			{
				mEncoders[decisionsPerFern * f + d].location.x = x;
				mEncoders[decisionsPerFern * f + d].location.y = y;
				mEncoders[decisionsPerFern * f + d].threshold = random_uniform01() * (colorBounds.y - colorBounds.x) + colorBounds.x;
			}
		}
	}
	else if(colorBounds.y < 1.0f)
	{
		//depth only
		for (int f = 0; f < mNumFerns; ++f)
		{
			int x = (int)floor(random_uniform01() * imgSize.x);
			int y = (int)floor(random_uniform01() * imgSize.y);
			for (int d = 0; d < decisionsPerFern; ++d)
			{
				mEncoders[decisionsPerFern * f + d].location.x = x;
				mEncoders[decisionsPerFern * f + d].location.y = y;
				mEncoders[decisionsPerFern * f + d].threshold = random_uniform01() * (depthBounds.y - depthBounds.x) + depthBounds.x;
			}
		}
	}
	else
	{
		//rgb-d
		for (int f = 0; f < mNumFerns; ++f)
		{
			int x = (int)floor(random_uniform01() * imgSize.x);
			int y = (int)floor(random_uniform01() * imgSize.y);
			for (int d = 0; d < decisionsPerFern; ++d)
			{
				mEncoders[decisionsPerFern * f + d].location.x = x;
				mEncoders[decisionsPerFern * f + d].location.y = y;

				if(d != 3)  mEncoders[decisionsPerFern * f + d].threshold = random_uniform01() * (colorBounds.y - colorBounds.x) + colorBounds.x;
				else mEncoders[decisionsPerFern * f + d].threshold = random_uniform01() * (depthBounds.y - depthBounds.x) + depthBounds.x;
			}
		}
	}
}

FernConservatory::~FernConservatory(void)
{
	delete[] mEncoders;
}

void FernConservatory::computeCode(const ORUtils::Image<float> *img, char *codeFragments) const
{
	const float *imgData = img->GetData(MEMORYDEVICE_CPU);
	for (int f = 0; f < mNumFerns; ++f)
	{
		codeFragments[f] = 0;
		int locId = mEncoders[f*mNumDecisions].location.x + mEncoders[f*mNumDecisions].location.y * img->noDims.x;
        float val = imgData[locId];

		for (int d = 0; d < mNumDecisions; ++d)
		{
			const FernTester *tester = &(mEncoders[f*mNumDecisions + d]);
			if (val <= 0.01f) codeFragments[f] = -1;
			else codeFragments[f] |= ((val < tester->threshold) ? 0 : 1) << d;
		}
	}
}

void FernConservatory::computeCode(const ORUtils::Image< ORUtils::Vector4<unsigned char> > *img, char *codeFragments) const
{
	const ORUtils::Vector4<unsigned char> *imgData = img->GetData(MEMORYDEVICE_CPU);
	for (int f = 0; f < mNumFerns; ++f)
	{
		codeFragments[f] = 0;
        int locId = mEncoders[f*mNumDecisions].location.x + mEncoders[f*mNumDecisions].location.y * img->noDims.x;

		for (int d = 0; d < mNumDecisions; ++d)
		{
			const FernTester *tester = &mEncoders[f * mNumDecisions + d];
			unsigned char tester_threshold = static_cast<unsigned char>(tester->threshold);

            unsigned char val = imgData[locId][d];
            codeFragments[f] |= ((val < tester_threshold) ? 0 : 1) << d;
		}
	}
}

void FernConservatory::computeCode(const ORUtils::Image<float> *depthImg, const ORUtils::Image< ORUtils::Vector4<unsigned char> > *colorImg, char *codeFragments) const
{
	const ORUtils::Vector4<unsigned char> *colorData = colorImg->GetData(MEMORYDEVICE_CPU);
	const float *depthData = depthImg->GetData(MEMORYDEVICE_CPU);
	for (int f = 0; f < mNumFerns; ++f)
	{
		codeFragments[f] = 0;
		int locId = mEncoders[f*mNumDecisions].location.x + mEncoders[f*mNumDecisions].location.y * depthImg->noDims.x;

		//d
		const FernTester *tester = &mEncoders[f * mNumDecisions + mNumDecisions-1];
		float val = depthData[locId];
//		if (val <= 0.01f){
//			codeFragments[f] = -1;
//			continue;
//		}
//		else codeFragments[f] |= ((val < tester->threshold) ? 0 : 1) << (mNumDecisions-1);
        codeFragments[f] |= ((val < tester->threshold) ? 0 : 1) << (mNumDecisions-1);
		//rgb
		for (int d = 0; d < mNumDecisions-1; ++d)
		{
			const FernTester *tester = &mEncoders[f * mNumDecisions + d];
			unsigned char tester_threshold = static_cast<unsigned char>(tester->threshold);

			unsigned char val = colorData[locId][d];
			codeFragments[f] |= ((val < tester_threshold) ? 0 : 1) << d;
		}

	}
}

void FernConservatory::SaveToFile(const std::string &fernsFileName)
{
	std::ofstream ofs(fernsFileName.c_str());

	if (!ofs) throw std::runtime_error("Could not open " + fernsFileName + " for reading");;

	for (int f = 0; f < mNumFerns * mNumDecisions; ++f)
		ofs << mEncoders[f].location.x << ' ' << mEncoders[f].location.y << ' ' << mEncoders[f].threshold << '\n';
}

void FernConservatory::LoadFromFile(const std::string &fernsFileName)
{
	std::ifstream ifs(fernsFileName.c_str());
	if (!ifs) throw std::runtime_error("unable to load " + fernsFileName);

	for (int i = 0; i < mNumFerns; i++)
	{
		for (int j = 0; j < mNumDecisions; j++)
		{
			FernTester &fernTester = mEncoders[i * mNumDecisions + j];
			ifs >> fernTester.location.x >> fernTester.location.y >> fernTester.threshold;
		}
	}
}
