#ifndef XSCANINTERFACE_H
#define XSCANINTERFACE_H

#include <ros/ros.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "xsens_parser.h"
#include "xsens_time_handler.h"

class PacketCallback;

class XsCanInterface
{
public:
    XsCanInterface();
    ~XsCanInterface();
    void initialize();
    void spinFor();
	void registerPublishers(ros::NodeHandle &node);
    void closeInterface();

private:
    int socket_;
    XsDataPacket packet;
    uint32_t m_startframeid; 
    uint32_t lastFrameId = 0xFFFFFFFF; // initialize with an invalid frame ID.
    std::list<PacketCallback *> m_callbacks;

    void registerCallback(PacketCallback *cb);
    void processCANMessages();
    void saveToPacket(const can_frame frame, XsDataPacket& packet);
    XsensTimeHandler m_timeHandler;
};

#endif // XSCANINTERFACE_H