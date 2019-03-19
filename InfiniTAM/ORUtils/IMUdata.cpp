//
// Created by zhuzunjie on 18-4-9.
//

#include "IMUdata.h"


namespace DataReader
{
//imu.gyrVar: 1.2e-3 #4e-3    # 1.2e-3
//imu.accVar: 8e-3   #2e-2    # 8.0e-3
//imu.gyrBiasVar: 4.0e-6    # 1e-9
//imu.accBiasVar: 4.0e-5    # 1e-8

// covariance of measurement
// From continuous noise_density of dataset sigma_g/sigma_a   rad/s/sqrt(Hz) -- m/s^2/sqrt(Hz)

/**
 * For EuRoc dataset, according to V1_01_easy/imu0/sensor.yaml
 * The params:
 * sigma_g: 1.6968e-4       rad / s / sqrt(Hz)
 * sigma_gw: 1.9393e-5      rad / s^2 / sqrt(Hz)
 * sigma_a: 2.0e-3          m / s^2 / sqrt(Hz)
 * sigma_aw: 3.0e-3         m / s^3 / sqrt(Hz)
 */

//// *1e3/*1e2 chosen by experiments
//    double IMUData::_gyrBiasRw2 = 2.0e-5*2.0e-5/**10*/;  //2e-12*1e3
//    double IMUData::_accBiasRw2 = 5.0e-3*5.0e-3/**10*/;  //4.5e-8*1e2
//
//// Try this?
////Matrix3d IMUData::_gyrMeasCov = Matrix3d::Identity()*2.0e-3*2.0e-3*200;//*8e-3*8e-3*200*100;//8e-3*8e-3*200*100;//2.5e-5*1e2;       // sigma_g * sigma_g / dt, ~5e-5*
////Matrix3d IMUData::_accMeasCov = Matrix3d::Identity()*8.0e-3*8.0e-3*200; //8e-3*8e-3*200*100;//8e-3*8e-3*200*100;//2.5e-4*1e3;       // sigma_a * sigma_a / dt, ~1.28e-4*
//    Matrix3d IMUData::_gyrMeasCov = Matrix3d::Identity()*1.7e-4*1.7e-4/0.005/**100*/;       // sigma_g * sigma_g / dt, ~6e-6*10
//    Matrix3d IMUData::_accMeasCov = Matrix3d::Identity()*2.0e-3*2.0e-3/0.005*100;       // sigma_a * sigma_a / dt, ~8e-4*10
//
//// covariance of bias random walk
//    Matrix3d IMUData::_gyrBiasRWCov = Matrix3d::Identity()*_gyrBiasRw2;     // sigma_gw * sigma_gw * dt, ~2e-12
//    Matrix3d IMUData::_accBiasRWCov = Matrix3d::Identity()*_accBiasRw2;     // sigma_aw * sigma_aw * dt, ~4.5e-8
//// Params used in Kalibr (generate a reasonable result)
//#continous
//accelerometer_noise_density: 0.02  #8e-4 * 10  --  0.01
//accelerometer_random_walk: 0.002  #1e-4 * 5  --  1-0.0002
//#continous
//gyroscope_noise_density: 0.01  #5e-4 * 10
//gyroscope_random_walk: 1.0e-04  #1e-5 * 5  --  1-4e-6

    IMUData::IMUData(const double& gx, const double& gy, const double& gz,
                     const double& ax, const double& ay, const double& az,
                     const double& t) :
            _g{gx,gy,gz}, _a{ax,ay,az}, _t(t)
    {
    }


//IMUData::IMUData(const IMUData& imu) :
//    _g(imu._g), _a(imu._a), _t(imu._t)
//{
//}

}
