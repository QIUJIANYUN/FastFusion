// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <fstream>
#include <iostream>
#include "FernConservatory.h"
#include "RelocDatabase.h"
#include "PoseDatabase.h"
#include "PixelUtils.h"

#include "../ORUtils/SE3Pose.h"

namespace FernRelocLib
{
    enum RelocType {DepthOnly, ColorOnly, Both};

	template <typename depthType, typename colorType>
	class Relocaliser
	{
	private:
		float keyframeHarvestingThreshold;
		FernConservatory *encoding;
		RelocDatabase *relocDatabase;
		PoseDatabase *poseDatabase;

		ORUtils::Image<depthType> *depthImage1, *depthImage2;
		ORUtils::Image<colorType> *rgbImage1, *rgbImage2;

		RelocType relocaType;

	public:
		Relocaliser(ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> range, float harvestingThreshold, int numFerns, int numDecisionsPerFern, RelocType type)
		{
			static const int levels = 4;
            relocaType = type;
			switch (type)
			{
                case DepthOnly:
                    encoding = new FernConservatory(numFerns, imgSize / (1 << levels), range, ORUtils::Vector2<float>(0,0), numDecisionsPerFern);

                case ColorOnly:
                    encoding = new FernConservatory(numFerns, imgSize / (1 << levels), ORUtils::Vector2<float>(0,0), ORUtils::Vector2<float>(0,255), 3);

                case Both:
                    encoding = new FernConservatory(numFerns, imgSize / (1 << levels), range, ORUtils::Vector2<float>(0,255), numDecisionsPerFern);

			    default:  std::cout<< "choose relocation type: depth, color, both"<<std::endl;
			}
            encoding->SaveToFile("../../ferns.txt");
			relocDatabase = new RelocDatabase(numFerns, encoding->getNumCodes());
			poseDatabase = new PoseDatabase();
			keyframeHarvestingThreshold = harvestingThreshold;

			depthImage1 = new ORUtils::Image<depthType>(imgSize, MEMORYDEVICE_CPU);
			depthImage2 = new ORUtils::Image<depthType>(imgSize, MEMORYDEVICE_CPU);
			rgbImage1 = new ORUtils::Image<colorType>(imgSize, MEMORYDEVICE_CPU);
			rgbImage2 = new ORUtils::Image<colorType>(imgSize, MEMORYDEVICE_CPU);
		}

		~Relocaliser(void)
		{
			delete encoding;
			delete relocDatabase;
			delete poseDatabase;
			delete depthImage1;
			delete depthImage2;
			delete rgbImage1;
			delete rgbImage2;
		}

        int ProcessFrame(const ORUtils::Image<depthType> *img, const ORUtils::SE3Pose *pose, int sceneId, int k, int nearestNeighbours[], float *distances, bool harvestKeyframes) const {
            // downsample and preprocess image => processedImage1
            filterSubsample(img, depthImage1); // 320x240
            filterSubsample(depthImage1, depthImage2); // 160x120
            filterSubsample(depthImage2, depthImage2); // 80x60
            filterSubsample(depthImage1, depthImage2); // 40x30

            filterGaussian(depthImage2, depthImage1, 2.5f);

            // compute code
            int codeLength = encoding->getNumFerns();
            char *code = new char[codeLength];
            encoding->computeCode(depthImage1, code);

            // prepare outputs
            int ret = -1;
            bool releaseDistances = (distances == NULL);
            if (distances == NULL) distances = new float[k];

            // find similar frames
            int similarFound = relocDatabase->findMostSimilar(code, nearestNeighbours, distances, k);

            // add keyframe to database
            if (harvestKeyframes) {
                if (similarFound == 0) ret = relocDatabase->addEntry(code);
                else if (distances[0] > keyframeHarvestingThreshold) ret = relocDatabase->addEntry(code);

                if (ret >= 0) poseDatabase->storePose(ret, *pose, sceneId);


            }

            // cleanup and return
            delete[] code;
            if (releaseDistances) delete[] distances;
            return ret;
        }

		int ProcessFrame(const ORUtils::Image<depthType> *img, const ORUtils::Image<colorType> *rgb, const ORUtils::SE3Pose *pose, int sceneId, int k, int nearestNeighbours[], float *distances, bool harvestKeyframes) const
		{
            // compute code
            int codeLength = encoding->getNumFerns();
            char *code = new char[codeLength];


		    switch (relocaType)
            {
                case DepthOnly:{
                    // downsample and preprocess image => depthImage1
                    filterSubsample(img, depthImage1); // 320x240
                    filterSubsample(depthImage1, depthImage2); // 160x120
                    filterSubsample(depthImage2, depthImage1); // 80x60
                    filterSubsample(depthImage1, depthImage2); // 40x30
                    filterGaussian(depthImage2, depthImage1, 2.5f);
                    encoding->computeCode(depthImage1, code);
                }
                case ColorOnly:{
                    //rgb
                    filterSubsample(rgb, rgbImage1); // 320x240
                    filterSubsample(rgbImage1, rgbImage2); // 160x120
                    filterSubsample(rgbImage2, rgbImage1); // 80x60
                    filterSubsample(rgbImage1, rgbImage2); // 40x30
                    filterGaussian(rgbImage2, rgbImage1, 2.5f);
                    encoding->computeCode(rgbImage1, code);
                }
                case Both:{
                    // downsample and preprocess image => depthImage1
                    filterSubsample(img, depthImage1); // 320x240
                    filterSubsample(depthImage1, depthImage2); // 160x120
                    filterSubsample(depthImage2, depthImage1); // 80x60
                    filterSubsample(depthImage1, depthImage2); // 40x30
                    filterGaussian(depthImage2, depthImage1, 2.5f);
                    //rgb
                    filterSubsample(rgb, rgbImage1); // 320x240
                    filterSubsample(rgbImage1, rgbImage2); // 160x120
                    filterSubsample(rgbImage2, rgbImage1); // 80x60
                    filterSubsample(rgbImage1, rgbImage2); // 40x30
                    filterGaussian(rgbImage2, rgbImage1, 2.5f);
                    encoding->computeCode(depthImage1, rgbImage1, code);
                }
            }

			// prepare outputs
			int ret = -1;
			bool releaseDistances = (distances == NULL);
			if (distances == NULL) distances = new float[k];

			// find similar frames
			int similarFound = relocDatabase->findMostSimilar(code, nearestNeighbours, distances, k);
//            cout << similarFound <<endl;
//            cout<< distances[0]<< " " << distances[1] <<endl;

			// add keyframe to database
			if (harvestKeyframes)
			{
				if (similarFound == 0)  ret = relocDatabase->addEntry(code);
				else if (distances[0] > keyframeHarvestingThreshold)    ret = relocDatabase->addEntry(code);

				if (ret >= 0) poseDatabase->storePose(ret, *pose, sceneId);
			}
			// cleanup and return
			delete[] code;
			if (releaseDistances) delete[] distances;
			return ret;
		}

		const FernRelocLib::PoseDatabase::PoseInScene & RetrievePose(int id)
		{
			return poseDatabase->retrievePose(id);
		}

		bool CheckOutlier()
		{
            return true;
		}

		void SaveToDirectory(const std::string& outputDirectory)
		{
			std::string configFilePath = outputDirectory + "config.txt";
			std::ofstream ofs(configFilePath.c_str());

			//TODO MAKE WORK WITH TEMPLATE - type should change?
			if (!ofs) throw std::runtime_error("Could not open " + configFilePath + " for reading");
			ofs << "type=rgb,levels=4,numFerns=" << encoding->getNumFerns() << ",numDecisionsPerFern=" << encoding->getNumDecisions() / 3 << ",harvestingThreshold=" << keyframeHarvestingThreshold;

			encoding->SaveToFile(outputDirectory + "ferns.txt");
			relocDatabase->SaveToFile(outputDirectory + "frames.txt");
			poseDatabase->SaveToFile(outputDirectory + "poses.txt");
		}

		void LoadFromDirectory(const std::string& inputDirectory)
		{
			std::string fernFilePath = inputDirectory + "ferns.txt";
			std::string frameCodeFilePath = inputDirectory + "frames.txt";
			std::string posesFilePath = inputDirectory + "poses.txt";

			if (!std::ifstream(fernFilePath.c_str())) throw std::runtime_error("unable to open " + fernFilePath);
			if (!std::ifstream(frameCodeFilePath.c_str())) throw std::runtime_error("unable to open " + frameCodeFilePath);
			if (!std::ifstream(posesFilePath.c_str())) throw std::runtime_error("unable to open " + posesFilePath);

			encoding->LoadFromFile(fernFilePath);
			relocDatabase->LoadFromFile(frameCodeFilePath);
			poseDatabase->LoadFromFile(posesFilePath);
		}

        int GetKeyframeNum()
        {
            return relocDatabase->GetEntryNum();
        }
	};
}

