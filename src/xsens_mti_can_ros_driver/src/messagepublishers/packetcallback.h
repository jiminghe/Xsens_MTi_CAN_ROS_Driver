#ifndef PACKETCALLBACK_H
#define PACKETCALLBACK_H

#include <ros/ros.h>
#include "xsens_parser.h"

const char* DEFAULT_FRAME_ID = "imu_link";

class PacketCallback
{
    public:
        virtual void operator()(const XsDataPacket &, ros::Time) = 0;
};

#endif