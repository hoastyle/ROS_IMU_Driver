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

#include "SocketComm.hpp"
#include <ros/ros.h>

SocketComm::SocketComm(std::string sensorName,
                       std::string ip,
                       uint8_t frameLength,
                       std::vector<uint8_t> header,
                       bool enableCheck) :
    mSensorName(sensorName),
    mSocketIP(ip),
    mFrameLength(frameLength),
    mvFrameHeader(header),
    mbEnableCheck(enableCheck),
    mbConnected(false),
    mbInitialized(false),
    mbHeaderReached(false),
    mbCacheCleared(false),
    mConnectTimes(0)
{
    ROS_INFO("[SocketComm] %s engine Constructor", sensorName.c_str());
    Init();
}

void SocketComm::socketConfig()
{
    // If this function reached, it means disconnection
    mbConnected = false;
    mbCacheCleared = false;
    mbHeaderReached = false;
    ROS_INFO("[SocketComm] %s socketConfig", mSensorName.c_str());
    mSocketHost = gethostbyname(mSocketIP.c_str());
    if(mSocketHost==NULL)
        ROS_INFO("[SocketComm] %s Fail to get host by name.", mSensorName.c_str());
    else
        ROS_INFO("[SocketComm] %s Success to get host by name ...", mSensorName.c_str());
    if((mSockfd=socket(AF_INET,SOCK_STREAM,0))==-1)
        ROS_INFO("[SocketComm] %s Fail to establish a socket", mSensorName.c_str());
    else
        ROS_INFO("[SocketComm] %s Success to establish a socket", mSensorName.c_str());

    /*init sockaddr_in*/
    mServAddr.sin_family=AF_INET;
    mServAddr.sin_port=htons(mServerPort);
    mServAddr.sin_addr=*((struct in_addr *)mSocketHost->h_addr);
    bzero(&(mServAddr.sin_zero),8);
    // set the recv() timeout
    struct timeval tv_timeout;
    tv_timeout.tv_sec = 1;
    tv_timeout.tv_usec = 0;
    setsockopt(mSockfd, SOL_SOCKET, SO_RCVTIMEO, &tv_timeout, sizeof(tv_timeout));
}

void SocketComm::Connect()
{
    if (mbConnected)
        return;

    ROS_INFO("[SocketComm] %s connect", mSensorName.c_str());
    while(!mbConnected)
    {
        socketConfig();
        if (mConnectTimes == 10)
        {
            sleep(10);
            mConnectTimes = 0;
            socketConfig();
        }
        if(connect(mSockfd,(struct sockaddr *)&mServAddr,sizeof(struct sockaddr_in))==-1)
        {
            ROS_INFO("[SocketComm] %s Fail to connect the socket", mSensorName.c_str());
            mConnectTimes ++;
            // usleep(500 * 1000);
        }
        else
        {
            ROS_INFO("[SocketComm] %s Success to connect the socket", mSensorName.c_str());
            mbConnected = true;
            mConnectTimes = 0;
        }
    }
}

void SocketComm::Init()
{
    ROS_INFO("[SocketComm] %s Init", mSensorName.c_str());
    mbInitialized = true;
}

void SocketComm::clearCache()
{
    if (mbCacheCleared)
        return;

    ROS_INFO("[SocketComm] %s clearCache", mSensorName.c_str());
    unsigned char Buffer[4096] = {0};
    int length = 4096;
    while (length == 4096)
    {
        length = recv(mSockfd, Buffer, 4096, 0);
        checkError(length);
    }
    mbCacheCleared = true;
}

void SocketComm::checkError(int length)
{
    if (length <= 0)
    {
        ROS_INFO("[SocketComm] %s connection error.", mSensorName.c_str());
        mbConnected = false;
        Connect();
    }
}

uint16_t SocketComm::Receive()
{
    Connect();
    clearCache();
    reachFrameHeader();
    memset(mSocketBuffer, mFrameLength, 0);
    int length = recv(mSockfd,mSocketBuffer,mFrameLength,0);
    checkError(length);

    uint16_t sumLength = length;
    while (sumLength < mFrameLength)
    {
        length = recv(mSockfd,(char *)&(mSocketBuffer[sumLength]),(mFrameLength - sumLength),0);
        checkError(length);
        sumLength = sumLength + length;
    }
    return sumLength;
}

bool SocketComm::readSocketData()
{
    if (!mbInitialized)
    {
        ROS_INFO("[SocketComm] %s readSocketData before Initialization.");
        return false;
    }
    ROS_INFO("[SocketComm] %s readSocketData.", mSensorName.c_str());
    uint16_t length = Receive();
    return checkFrame(length);
}

void SocketComm::reachFrameHeader()
{
    if (mbHeaderReached)
        return;

    ROS_INFO("[SocketComm] %s reachFrameHeader", mSensorName.c_str());
    if (mvFrameHeader.size() == 2)
    {
        while (1)
        {
            uint8_t buffer[100] = {0};
            int length = recv(mSockfd, buffer,2,0);
            checkError(length);
            if (buffer[0] == mvFrameHeader[0] && buffer[1] == mvFrameHeader[1])
            {
                length = recv(mSockfd,buffer,mFrameLength - 2,0);
                break;
            }
            else if (buffer[0] != mvFrameHeader[0] && buffer[1] == mvFrameHeader[0])
            {
                length = recv(mSockfd,buffer,mFrameLength - 1,0);
                break;
            }
            else if (buffer[0] == mvFrameHeader[1] && buffer[1] != mvFrameHeader[1])
            {
                length = recv(mSockfd,buffer,mFrameLength - 3,0);
                break;
            }
        }
        mbHeaderReached = true;
        return;
    }
}

bool SocketComm::checkFrame(uint16_t length)
{
    if (!mbEnableCheck)
        return true;
    if (length != mFrameLength)
        return false;
    for (uint8_t i = 0; i < mvFrameHeader.size(); i ++)
        if (mSocketBuffer[i] != mvFrameHeader[i])
            return false;
    return true;
}

void SocketComm::Close()
{
    close(mSockfd);
}
