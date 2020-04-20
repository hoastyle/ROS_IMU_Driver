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
#include "IMUEngine.hpp"

#define PI 3.1415926535898

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icrane_imu");
    ros::NodeHandle node;
    ros::Publisher imuPub = node.advertise<sensor_msgs::Imu>("imu", 20);
    ros::Rate loop_rate(100);

    std::string yamlFile = CONFIG_PATH "imu_device.yaml";
    YAML::Node imuConfig = YAML::LoadFile(yamlFile);

    std::string ip = imuConfig["ip"].as<std::string>();
    MM_IMU_DATA data;
    boost::shared_ptr<IMUEngine> pImu(new IMUEngine(ip));
    pImu->Init();

    while(ros::ok())
    {
        ROS_INFO_STREAM("test----------------");
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

        imuPub.publish(imu_data);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
