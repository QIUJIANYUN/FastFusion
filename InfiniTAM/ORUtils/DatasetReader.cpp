//
// Created by zhuzunjie on 18-4-9.
//
#include "DatasetReader.h"
#include <iostream>
#include <dirent.h>
namespace DataReader {
    void loadImageList(const char *imgPath, const char *associate, std::vector<ICell> &colorList, std::vector<ICell> &depthList, int datasetMode) {

        ifstream inf;
        inf.open(associate, ifstream::in);

        string line;
        size_t comma = 0;
        size_t comma2 = 0;
        ICell temp;
        int number_of_frames = 0;

        switch(datasetMode){
            case DatasetMode::ICL :
                DIR *dp;
                struct dirent *dirp;

                if((dp  = opendir(imgPath)) != NULL){
                    while ((dirp = readdir(dp)) != NULL) {
                        std::string name = std::string(dirp->d_name);

                        if (name != "." && name != "..")
                            number_of_frames++;
                    }
                    closedir(dp);
                }
                for(int count = 0; count < number_of_frames-1; count++){
                    temp.timeStamp = double(count) * double(1.0/30.0);
                    if(temp.timeStamp < 1e-3) continue;
                    temp.imgName = to_string(count) + ".png";
                    colorList.push_back(temp);
                    depthList.push_back(temp);
                }

                break;
            case DatasetMode::TUM :
                while (!inf.eof()) {
                    getline(inf, line);

                    comma = line.find(' ', 0);
                    double timestamp = (double) atof(line.substr(0, comma).c_str());
                    if(timestamp < 1e-3) continue;

                    comma2 = line.find(' ', comma + 1);
                    string colorName = line.substr(comma + 1, comma2 - comma-1).c_str();

                    comma = line.find(' ', comma2+1);
                    string temp1 = line.substr(comma2+1, comma-comma2-1);
                    timestamp = (double) atof(temp1.c_str());

                    comma2 = line.find('g', comma + 1);
                    string depthName = line.substr(comma + 1, comma2 - comma).c_str();

                    temp.timeStamp = timestamp;
                    if(temp.timeStamp < 1e-3) continue;
                    temp.imgName = colorName;
                    colorList.push_back(temp);
                    temp.imgName = depthName;
                    depthList.push_back(temp);
                }

                break;
            case DatasetMode::MyZR300 :
            case DatasetMode::MyD435i :
            case DatasetMode::MyAzureKinect :
                getline(inf, line);
                while (!inf.eof()) {
                    getline(inf, line);

                    comma = line.find(',', 0);
                    string temp1 = line.substr(0, comma);
                    double timestamp =  (double) atof(temp1.c_str());
                    if(timestamp < 1e-3) continue;
                    comma2 = line.find('g', comma + 1);
                    string imgName = line.substr(comma + 1, comma2 - comma).c_str();

                    temp.timeStamp = timestamp * 1e-6;
                    temp.imgName = imgName;
                    colorList.push_back(temp);
                    depthList.push_back(temp);
                }
                break;
            default: //my dataset, i.e. the color and depth image have same timestamp(name).
                break;
        }

        inf.close();
    }

    void loadIMUFile(const char *imuPath, std::vector<IMUData> &vimuData) {
        ifstream inf;
        inf.open(imuPath, ifstream::in);
        if(!inf){
            cout<< "No IMU information" << endl;
            return;
        }
        const int cnt = 7;          // 你要输出的个数

        string line;
        //int i = 0;
        int j = 0;
        size_t comma = 0;
        size_t comma2 = 0;

        //     char imuTime[14] = {0};
        double acc[3] = {0.0};
        double grad[3] = {0.0};
        double imuTimeStamp = 0.0;
        getline(inf, line);
        while (!inf.eof()) {
            getline(inf, line);
            comma = line.find(',', 0);
            string temp = line.substr(0, comma);
            imuTimeStamp = (double) atof(temp.c_str());

            //cout<<line.substr(0,comma).c_str()<<' ';
            //memcpy(imuTimeStamp,line.substr(0,comma).c_str(),line.substr(0,comma).length);
            while (comma < line.size() && j != cnt - 1) {

                comma2 = line.find(',', comma + 1);
                switch (j) {
                    case 0:
                        grad[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                        break;
                    case 1:
                        grad[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                        break;
                    case 2:
                        grad[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                        break;
                    case 3:
                        acc[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                        break;
                    case 4:
                        acc[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                        break;
                    case 5:
                        acc[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                        break;
                }
                //cout<<line.substr(comma + 1,comma2-comma-1).c_str()<<' ';
                ++j;
                comma = comma2;
            }
            if(imuTimeStamp < 1e-3) continue;
//            IMUData tempImu(grad[0], grad[1], grad[2], acc[0], acc[1], acc[2], imuTimeStamp + 0.00892/* * 1e-6*/); //eth3d
            IMUData tempImu(grad[0], grad[1], grad[2], acc[0], acc[1], acc[2], imuTimeStamp  * 1e-6);
            vimuData.push_back(tempImu);

            j = 0;
        }

        inf.close();

        //return 0;
    }

    void synInit(std::vector<ICell> &imageList, std::vector<IMUData> &imuList, int imgIdx, int imuIdx) {
        // Find Start Idx
        // 预处理数据
        int startImuIdx = 0;
        int startImageIdx = 0;
        // 剔除初始的冗余IMU数据
        while (1) {
            if (imuList[startImuIdx]._t >= imageList[0].timeStamp)
                break;

            startImuIdx++;
        }

        // 剔除初始的冗余Image数据
        while (1) {
            if (imuList[0]._t <= imageList[startImageIdx].timeStamp)
                break;

            startImageIdx++;
        }
        // 将IMU和图片时间戳对齐
        while (1) {
            if (imuList[startImuIdx]._t >= imageList[startImageIdx].timeStamp)
                break;

            startImuIdx++;
        }
        imgIdx = startImageIdx;
        imuIdx = startImuIdx;
    }
}

/*void loadGTFile(const char *imuPath, std::vector<GT> &vGTData) {
    ifstream inf;
    inf.open(imuPath, ifstream::in);
    const int cnt = 7;          // 你要输出的个数

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;

    getline(inf, line);
    while (!inf.eof()) {
        GT tempGT;
        getline(inf, line);
        comma = line.find(',', 0);
        string temp = line.substr(0, comma);
        tempGT.timeStamp = (double) atof(temp.c_str());

        //cout<<line.substr(0,comma).c_str()<<' ';
        //memcpy(imuTimeStamp,line.substr(0,comma).c_str(),line.substr(0,comma).length);
        while (j < cnt) {

            comma2 = line.find(',', comma + 1);
            switch (j) {
                case 0:
                    tempGT.position[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 1:
                    tempGT.position[0] = -(atof(line.substr(comma + 1, comma2 - comma - 1).c_str()));
                    break;
                case 2:
                    tempGT.position[1] = -(atof(line.substr(comma + 1, comma2 - comma - 1).c_str()));
                    break;
                    // 				case 3:
                    // 					tempGT.rotation_Q[0] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
                    // 					break;
                    // 				case 4:
                    // 					tempGT.rotation_Q[1] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
                    // 					break;
                    // 				case 5:
                    // 					tempGT.rotation_Q[2] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
                    // 					break;
                    // 				case 6:
                    // 					tempGT.rotation_Q[3] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
                    // 					break;
            }
            //cout<<line.substr(comma + 1,comma2-comma-1).c_str()<<' ';
            ++j;
            comma = comma2;
        }

        vGTData.push_back(tempGT);

        j = 0;
    }

    inf.close();

    //return 0;
}*/