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

#ifndef IMU_ENGINE_HPP
#define IMU_ENGINE_HPP

#include <yaml.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "iostream"
#include "icrane_imu.hpp"
#include "SocketComm.hpp"

class IMUEngine
{
private:
    uint16_t mDataLength;
    std::string mIP;
    uint32_t mIndex;
    // time sycn
    bool mbSynEnable;
    uint64_t mDeltaTime;
    uint64_t mCount;
    uint16_t mSynInterval;
    MM_UTCTime mUTCTime;
    boost::mutex mMutexSyn;
    void readConfig();
    void parseFrame(MM_IMU_DATA &imu);
    boost::shared_ptr<SocketComm> mpSocket;

public:
    IMUEngine(std::string ip);
    void Init();
    void Start();
    void Stop();
    bool readIMUData(MM_IMU_DATA &imu);
    // void synTime();
};

#endif //IMU_ENGINE_HPP
