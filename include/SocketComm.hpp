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

#ifndef SOCKET_COMM_HPP
#define SOCKET_COMM_HPP

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fstream>
#include <unistd.h>
#include <vector>

class SocketComm
{
private:
    std::string mSensorName;
    std::string mSocketIP;
    uint16_t mFrameLength;
    std::vector<uint8_t> mvFrameHeader;
    uint8_t mHeaderSize;
    bool mbEnableCheck;

    bool mbConnected;
    bool mbInitialized;
    bool mbHeaderReached;
    bool mbCacheCleared;

    struct hostent * mSocketHost;
    struct sockaddr_in mServAddr;
    const uint16_t mServerPort = 4196;
    int mConnectTimes;

    void Init();
    void socketConfig();
    void clearCache();
    void reachFrameHeader();
    uint16_t Receive();
    void checkError(int length);
    bool checkFrame(uint16_t length);

public:
    int mSockfd;
    unsigned char mSocketBuffer[1000] = {0};
    SocketComm(std::string sensorName, std::string ip, uint8_t frameLength, \
                std::vector<uint8_t> header, bool enableCheck);
    void Connect();
    bool readSocketData();
    void Close();
};

#endif
