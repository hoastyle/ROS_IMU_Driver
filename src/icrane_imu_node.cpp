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

#include <ctime>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "icrane_imu.hpp"

IMUEngine::IMUEngine(std::string ip):
    mIP(ip),
    mIndex(0)
{
    printf("[IMUEngine] Constructor");
}

void IMUEngine::Init()
{
    mDeltaTime = 0;
    mRetry = REMOVE_CACHE;
    mCount = 0;
    mFailureTimes = 0;
    initComm();
    readConfig();
}

void IMUEngine::readConfig()
{
    std::string yamlFile = "/home/hao/Workspace/SLAM/IMU/ws/src/icrane_imu/imu_device.yaml";
    YAML::Node imuConfig = YAML::LoadFile(yamlFile);

    mSynInterval = imuConfig["synInterval"].as<unsigned int>();
    mbSynEnable = imuConfig["timeSynEnable"].as<bool>();
    mDataLength = imuConfig["dataLength"].as<unsigned int>();
    ROS_INFO_STREAM("[IMUEngine] YAML: mSynInterval is " << mSynInterval << " mbSynEnable is " << mbSynEnable);
}

void IMUEngine::initComm()
{
    configureComm();
    if(connect(mSockfd,(struct sockaddr *)&mServAddr,sizeof(struct sockaddr_in))==-1)
    {
        printf("[IMUEngine] Fail to connect the socket");
    }
    else
    {
        printf("[IMUEngine] Success to connect the socket...");
        mbIsInitialized = true;
    }
}

bool IMUEngine::readIMUData(MM_IMU_DATA &imu)
{
    if (mbIsInitialized)
    {
        // empty the socket cache
        if (mRetry == REMOVE_CACHE)
        {
            unsigned char Buffer[4096] = {0};
            int length = recv(mSockfd, Buffer, 4096, 0);
            while (length == 4096)
            {
                length = recv(mSockfd, Buffer, 4096, 0);
            }
            mRetry = COMPLETE_PACK;
        }
        // clear the buffer
        for (int i = 0; i < mDataLength; ++i) {
            mIMUBuffer[i] = 0;
        }
        if (mRetry == COMPLETE_PACK)
        {
            while (1)
            {
                int length = recv(mSockfd, mIMUBuffer,2,0);
                if (length <= 0)
                {
                    mFailureTimes ++;
                    if (mFailureTimes >= 5)
                    {
                        mFailureTimes = 0;
                        restoreConnection();
                        mRetry = REMOVE_CACHE;
                        return false;
                    }
                    continue;
                }
                if (mIMUBuffer[0] == 0x55 && mIMUBuffer[1] == 0x55)
                {
                    length = recv(mSockfd,mIMUBuffer,mDataLength - 2,0);
                    break;
                }
                else if (mIMUBuffer[0] != 0x55 && mIMUBuffer[1] == 0x55)
                {
                    length = recv(mSockfd,mIMUBuffer,mDataLength - 1,0);
                    break;
                }
                else if (mIMUBuffer[0] == 0x55 && mIMUBuffer[1] != 0x55)
                {
                    length = recv(mSockfd,mIMUBuffer,mDataLength - 3,0);
                    break;
                }
            }
            mRetry = IMU_DATA_DONE;
            return false;
        }
        if (mRetry == IMU_DATA_DONE)
        {
            int length = recv(mSockfd,mIMUBuffer,mDataLength,0);
            if (length < 0)
            {
                mRetry = COMPLETE_PACK;
                return false;
            }
            int sumLength = length;
            while (sumLength < mDataLength)
            {
                length = recv(mSockfd,(char *)&(mIMUBuffer[sumLength]),(mDataLength - sumLength),0);
                if (length < 0)
                {
                    mRetry = COMPLETE_PACK;
                    return false;
                }
                sumLength = sumLength + length;
            }
            if(!checkIMUData(sumLength)) {
                mRetry = COMPLETE_PACK;
                return false;
            }
        }

        // TODO: thread safe
        ++mIndex;
        imu.index = mIndex;
        uint32_t temp = (uint32_t)((mIMUBuffer[8] << 24) + (mIMUBuffer[7] << 16) + (mIMUBuffer[6] << 8) + mIMUBuffer[5]);
        imu.clock = temp;

        temp = (unsigned int)((mIMUBuffer[20] << 24) + (mIMUBuffer[19] << 16) + (mIMUBuffer[18] << 8) + mIMUBuffer[17]);
        imu.roll = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[24] << 24) + (mIMUBuffer[23] << 16) + (mIMUBuffer[22] << 8) + mIMUBuffer[21]);
        imu.pitch = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[28] << 24) + (mIMUBuffer[27] << 16) + (mIMUBuffer[26] << 8) + mIMUBuffer[25]);
        imu.heading = *(float *)&temp;

        temp = (unsigned int)((mIMUBuffer[32] << 24) + (mIMUBuffer[31] << 16) + (mIMUBuffer[30] << 8) + mIMUBuffer[29]);
        imu.XRate = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[36] << 24) + (mIMUBuffer[35] << 16) + (mIMUBuffer[34] << 8) + mIMUBuffer[33]);
        imu.YRate = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[40] << 24) + (mIMUBuffer[39] << 16) + (mIMUBuffer[38] << 8) + mIMUBuffer[37]);
        imu.ZRate = *(float *)&temp;

        temp = (unsigned int)((mIMUBuffer[44] << 24) + (mIMUBuffer[43] << 16) + (mIMUBuffer[42] << 8) + mIMUBuffer[41]);
        imu.XAccel = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[48] << 24) + (mIMUBuffer[47] << 16) + (mIMUBuffer[46] << 8) + mIMUBuffer[45]);
        imu.YAccel = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[52] << 24) + (mIMUBuffer[51] << 16) + (mIMUBuffer[50] << 8) + mIMUBuffer[49]);
        imu.ZAccel = *(float *)&temp;

        temp = (unsigned int)((mIMUBuffer[56] << 24) + (mIMUBuffer[55] << 16) + (mIMUBuffer[54] << 8) + mIMUBuffer[53]);
        imu.XVelocity = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[60] << 24) + (mIMUBuffer[59] << 16) + (mIMUBuffer[58] << 8) + mIMUBuffer[57]);
        imu.midHeading = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[64] << 24) + (mIMUBuffer[63] << 16) + (mIMUBuffer[62] << 8) + mIMUBuffer[61]);
        imu.offset = *(float *)&temp;
        temp = (unsigned int)((mIMUBuffer[68] << 24) + (mIMUBuffer[67] << 16) + (mIMUBuffer[66] << 8) + mIMUBuffer[65]);
        imu.position = *(float *)&temp;

        if (mCount % (mSynInterval * 100) == 0)
            synTime();
        mCount ++;

        imu.timestamp = imu.clock + mDeltaTime;

        printf("Roll: %f Pitch: %f\n", imu.roll, imu.pitch);
        printf("%f %f %f %f %f %f\n", imu.XRate, imu.YRate, imu.ZRate, imu.XAccel, imu.YAccel, imu.ZAccel);
        printf("Clock: %u, Timestamp: %lf\n", imu.clock, imu.timestamp);
        printf("The parsed IMU data is %f %f %f %f\n", imu.XVelocity, imu.midHeading, imu.offset, imu.position);
        return true;
    }
    else
    {
        return false;
    }
}

void IMUEngine::synTime()
{
    // boost::unique_lock<boost::mutex> lock(mMutexSyn);
    // combineUTCTime(mUTCTime, DEVICE_IMU);
    // mDeltaTime = mUTCTime.milliUTC - mUTCTime.triIMU * 1000 * 10;  // a pulse every 10 seconds, time 0 has no pulse
    // printf("[IMUEngine] IMU Time synchronization. Delta Time between UTCTime and local time is %llu ms.", mDeltaTime);
}

bool IMUEngine::checkIMUData(int length)
{
    if (mIMUBuffer[0] == 0x55 && mIMUBuffer[1] == 0x55 && mIMUBuffer[2] == 0x61 && mIMUBuffer[3] == 0x32 && length == mDataLength)
    {
        return true;
    }
    return false;
}

bool IMUEngine::isInitialized()
{
    return mbIsInitialized;
}

void IMUEngine::restoreConnection()
{
    printf("[IMUEngine] Resotoring IMU connection..");
    configureComm();
    int times = 1;
    while (1)
    {
        printf("[IMUEngine] Try connecting the %d times", times);
        int status = connect(mSockfd,(struct sockaddr *)&mServAddr,sizeof(struct sockaddr_in));
        sleep(1);
        if (status == -1)
        {
            times ++;
            continue;
        }
        else
        {
            printf("[IMUEngine] Reconnection succeed");
            return;
        }
    }
}

void IMUEngine::Stop()
{
    printf("[IMUEngine] Stop");
    close(mSockfd);
}

void IMUEngine::Start()
{
    close(mSockfd);
    initComm();
}

void IMUEngine::configureComm()
{
    printf("[IMUEngine] Init");
    mHost = gethostbyname(mIP.c_str());
    if(mHost==NULL)
    {
        printf("[IMUEngine] Fail to get host by name.");
    }
    printf("[IMUEngine] Success to get host by name ...");

    if((mSockfd=socket(AF_INET,SOCK_STREAM,0))==-1)
    {
        printf("[IMUEngine] Fail to establish a socket");
    }
    printf("[IMUEngine] Success to establish a socket...");

    /*init sockaddr_in*/
    mServAddr.sin_family=AF_INET;
    mServAddr.sin_port=htons(mServerPort);
    mServAddr.sin_addr=*((struct in_addr *)mHost->h_addr);
    bzero(&(mServAddr.sin_zero),8);
    // set the recv() timeout
    struct timeval tv_timeout;
    tv_timeout.tv_sec = 1;
    tv_timeout.tv_usec = 0;
    setsockopt(mSockfd, SOL_SOCKET, SO_RCVTIMEO, &tv_timeout, sizeof(tv_timeout));
}

#define PI 3.1415926535898

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icrane_imu");
    ros::NodeHandle node;
    ros::Publisher imuPub = node.advertise<sensor_msgs::Imu>("imu", 20);
    ros::Rate loop_rate(50);

    std::string yamlFile = "/home/hao/Workspace/SLAM/IMU/ws/src/icrane_imu/imu_device.yaml";
    YAML::Node imuConfig = YAML::LoadFile(yamlFile);

    std::string ip = imuConfig["ip"].as<std::string>();

    MM_IMU_DATA data;
    boost::shared_ptr<IMUEngine> pImu(new IMUEngine(ip));
    pImu->Init();

    while(ros::ok())
    {
        sensor_msgs::Imu imu_data;
        pImu->readIMUData(data); 
        imu_data.header.stamp = ros::Time::now(); 
        imu_data.header.frame_id = "imu_link";
        imu_data.linear_acceleration.x = data.XAccel;
        imu_data.linear_acceleration.y = data.YAccel;
        imu_data.linear_acceleration.z = data.ZAccel;
        imu_data.angular_velocity.x = data.XRate * PI / 180;
        imu_data.angular_velocity.y = data.YRate * PI / 180;
        imu_data.angular_velocity.z = data.ZRate * PI / 180;
        ROS_INFO_STREAM(data.XRate);
        ROS_INFO_STREAM(data.YRate);
        ROS_INFO_STREAM(data.ZRate);

        imuPub.publish(imu_data);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
