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

#ifndef ICRANE_IMU_HPP
#define ICRANE_IMU_HPP 

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
#include <yaml.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "iostream"

typedef enum
{
    REMOVE_CACHE = 0,
    COMPLETE_PACK = 1,
    IMU_DATA_DONE = 2
} MM_IMU_ENGINE_STATUS;

typedef struct             
{                          
    uint32_t milliTy;      
    uint16_t triTy;        
    uint32_t milliIMU;     
    uint16_t triIMU;       
    uint64_t milliUTC;     
} MM_UTCTime;              

typedef struct
{
    uint32_t clock;
    float roll;
    float pitch;
    float heading;
    float XRate;
    float YRate;
    float ZRate;
    float XAccel;
    float YAccel;
    float ZAccel;
    float XVelocity;
    float midHeading;
    float offset;
    float position;
    double timestamp;
    uint32_t index;
} MM_IMU_DATA;

#endif //ICRANE_IMU_HPP
