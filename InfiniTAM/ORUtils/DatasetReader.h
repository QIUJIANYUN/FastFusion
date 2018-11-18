//
// Created by zhuzunjie on 18-4-9.
//

#ifndef INFINITAM_DATASETREADER_H
#define INFINITAM_DATASETREADER_H

#include <vector>
#include <string>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "IMUdata.h"

using namespace std;
namespace DataReader {
    typedef struct ImageList {
        double timeStamp;
        string imgName;
    } ICell;

    typedef struct loadGroundtruth {
        double timeStamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond rotation_Q;


    } GT;

    void loadImageList(const char *imagePath, std::vector<ICell> &iListData);

    void loadIMUFile(const char *imuPath, std::vector<IMUData> &vimuData);

/*
 * @brief synchronize init time of imu and image.
 */
    void synInit(std::vector<ICell> &imageList, std::vector<IMUData> &imuList, int imgIdx, int imuIdx);

/*
 * @brief read groundtruth information.
 * TODO: from now on we only read position, need to read orientation as well.
 */
    void loadGTFile(const char *imuPath, std::vector<GT> &vGTData);

}



#endif //INFINITAM_DATASETREADER_H
