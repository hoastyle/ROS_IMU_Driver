/*
 * SHANGHAI MASTER MATRIX CONFIDENTIAL
 * Copyright 2018 Shanghai Master Matrix Corporation All Rights Reserved.

 * The source code, information and material ("Material") contained herein is owned
 * by Shanghai Master Matrix Corporation or its suppliers or licensors, and title to such
 * Material remains with Shanghai Master Matrix Corporation or its suppliers or licensors.
 * The Material contains proprietary information of Shanghai Master Matrix or its suppliers
 * and licensors. The Material is protected by worldwide copyright laws and treaty provisions.
 * No part of the Material may be used, copied, reproduced, modified, published, uploaded,
 * posted, transmitted, distributed or disclosed in any way without Shanghai Master Matrix's
 * prior express written permission. No license under any patent, copyright or other intellectual
 * property rights in the Material is granted to or conferred upon you, either expressly,
 * by implication, inducement, estoppel or otherwise. Any license under such intellectual
 * property rights must be express and approved by Shanghai Master Matrix in writing.

 * Unless otherwise agreed by Shanghai Master Matrix in writing, you may not remove or alter
 * this notice or any other notice embedded in Materials by Shanghai Master Matrix or
 * Shanghai Master Matrix's suppliers or licensors in any way.
 */

#include "time.h"
#include <ctime>
#include <cmath>
#include "IMUEngine.hpp"
#include "ros/ros.h"

IMUEngine::IMUEngine(std::string ip) :
    mIP(ip)
{
    ROS_INFO("[IMUEngine] Constructor\n");
    readConfig();
}

void IMUEngine::readConfig()
{
    std::string yamlFile = CONFIG_PATH "imu_device.yaml";
    ROS_INFO_STREAM(yamlFile);
    YAML::Node imuConfig = YAML::LoadFile(yamlFile);

    mSynInterval = imuConfig["synInterval"].as<unsigned int>();
    mbSynEnable = imuConfig["timeSynEnable"].as<bool>();
    mDataLength = imuConfig["dataLength"].as<unsigned int>();
    ROS_INFO("[IMUEngine] YAML: mSynInterval is %u, mbSynEnable is %d.\n", mSynInterval, mbSynEnable);
}

void IMUEngine::Init()
{
    ROS_INFO("[IMUEngine] Init\n");
    std::vector<uint8_t> vec = {0x55, 0x55};
    mpSocket = boost::make_shared<SocketComm>("IMUEngine", mIP, mDataLength, vec, true);
    mDeltaTime = 0;
    mCount = 0;
    mIndex = 0;
}

void IMUEngine::Stop()
{
    ROS_INFO("[IMUEngine] Stop\n");
    mpSocket->Close();
}

void IMUEngine::Start()
{
    ROS_INFO("[IMUEngine] Start\n");
    mpSocket->Close();
}

bool IMUEngine::readIMUData(MM_IMU_DATA &imu)
{
    if (!mpSocket->readSocketData())
        return false;
    parseFrame(imu);
    // if (mbSynEnable)
    // {
        // if (mCount % (mSynInterval * 100) == 0)
            // synTime();
        // mCount ++;
        // imu.timestamp = imu.clock + mDeltaTime;
    // }

    return true;
}

void IMUEngine::parseFrame(MM_IMU_DATA &imu)
{
    ++mIndex;
    imu.index = mIndex;
    uint32_t temp = (uint32_t)((mpSocket->mSocketBuffer[8]<<24) + (mpSocket->mSocketBuffer[7]<<16) \
                    + (mpSocket->mSocketBuffer[6]<<8) + mpSocket->mSocketBuffer[5]);
    imu.clock = temp;

    temp = (unsigned int)((mpSocket->mSocketBuffer[20]<<24) + (mpSocket->mSocketBuffer[19]<<16) \
                    + (mpSocket->mSocketBuffer[18]<<8) + mpSocket->mSocketBuffer[17]);
    imu.roll = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[24]<<24) + (mpSocket->mSocketBuffer[23]<<16) \
                    + (mpSocket->mSocketBuffer[22]<<8) + mpSocket->mSocketBuffer[21]);
    imu.pitch = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[28]<<24) + (mpSocket->mSocketBuffer[27]<<16) \
                    + (mpSocket->mSocketBuffer[26]<<8) + mpSocket->mSocketBuffer[25]);
    imu.heading = *(float *)&temp;

    temp = (unsigned int)((mpSocket->mSocketBuffer[32]<<24) + (mpSocket->mSocketBuffer[31]<<16) \
                    + (mpSocket->mSocketBuffer[30]<<8) + mpSocket->mSocketBuffer[29]);
    imu.XRate = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[36]<<24) + (mpSocket->mSocketBuffer[35]<<16) \
                    + (mpSocket->mSocketBuffer[34]<<8) + mpSocket->mSocketBuffer[33]);
    imu.YRate = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[40]<<24) + (mpSocket->mSocketBuffer[39]<<16) \
                    + (mpSocket->mSocketBuffer[38]<<8) + mpSocket->mSocketBuffer[37]);
    imu.ZRate = *(float *)&temp;

    temp = (unsigned int)((mpSocket->mSocketBuffer[44]<<24) + (mpSocket->mSocketBuffer[43]<<16) \
                    + (mpSocket->mSocketBuffer[42]<<8) + mpSocket->mSocketBuffer[41]);
    imu.XAccel = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[48]<<24) + (mpSocket->mSocketBuffer[47]<<16) \
                    + (mpSocket->mSocketBuffer[46]<<8) + mpSocket->mSocketBuffer[45]);
    imu.YAccel = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[52]<<24) + (mpSocket->mSocketBuffer[51]<<16) \
                    + (mpSocket->mSocketBuffer[50]<<8) + mpSocket->mSocketBuffer[49]);
    imu.ZAccel = *(float *)&temp;

    temp = (unsigned int)((mpSocket->mSocketBuffer[56]<<24) + (mpSocket->mSocketBuffer[55]<<16) \
                    + (mpSocket->mSocketBuffer[54]<<8) + mpSocket->mSocketBuffer[53]);
    imu.XVelocity = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[60]<<24) + (mpSocket->mSocketBuffer[59]<<16) \
                    + (mpSocket->mSocketBuffer[58]<<8) + mpSocket->mSocketBuffer[57]);
    imu.midHeading = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[64]<<24) + (mpSocket->mSocketBuffer[63]<<16) \
                    + (mpSocket->mSocketBuffer[62]<<8) + mpSocket->mSocketBuffer[61]);
    imu.offset = *(float *)&temp;
    temp = (unsigned int)((mpSocket->mSocketBuffer[68]<<24) + (mpSocket->mSocketBuffer[67]<<16) \
                    + (mpSocket->mSocketBuffer[66]<<8) + mpSocket->mSocketBuffer[65]);
    imu.position = *(float *)&temp;

    ROS_INFO("The IMU data is %f %f %f %f %f %f %f %f %f\n",
        imu.roll, imu.pitch, imu.heading, imu.XRate, imu.YRate, imu.ZRate,
        imu.XAccel, imu.YAccel, imu.ZAccel);
    ROS_INFO("The IMU data timestamp is %u %llu\n", imu.clock, imu.timestamp);
    ROS_INFO("The parsed IMU data is %f %f %f %f\n",
        imu.XVelocity, imu.midHeading, imu.offset, imu.position);
}

// void IMUEngine::synTime()
// {
    // boost::unique_lock<boost::mutex> lock(mMutexSyn);
    // combineUTCTime(mUTCTime, DEVICE_IMU);
    // mDeltaTime = mUTCTime.milliUTC - mUTCTime.triIMU * 1000 * 10;  // a pulse every 10 seconds, time 0 has no pulse
    // ROS_INFO("[IMUEngine] IMU Time synchronization. Delta Time between UTCTime and local time is %llu ms.", mDeltaTime);
// }
