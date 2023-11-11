
//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "packetcallback.h"
#include "xsens_mti_can_ros_driver/XsStatusWord.h"

struct StatusPublisher : public PacketCallback
{
    ros::Publisher pub;

    StatusPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<xsens_mti_can_ros_driver::XsStatusWord>("status", pub_queue_size);
    }

    void assignMessage(xsens_mti_can_ros_driver::XsStatusWord &msg, XsStatusWord status)
    {

        msg.selftest = status.selftest;
        msg.filter_valid = status.filter_valid;
        msg.gnss_fix = status.gnss_fix;
        msg.no_rotation_update_status = status.no_rotation_update_status; 
        msg.representative_motion = status.representative_motion ;
        msg.clock_bias_estimation = status.clock_bias_estimation ;
        msg.clipflag_acc_x = status.clipflag_acc_x ;
        msg.clipflag_acc_y = status.clipflag_acc_y;
        msg.clipflag_acc_z = status.clipflag_acc_z;
        msg.clipflag_gyr_x = status.clipflag_gyr_x;
        msg.clipflag_gyr_y = status.clipflag_gyr_y;
        msg.clipflag_gyr_z = status.clipflag_gyr_z;
        msg.clipflag_mag_x = status.clipflag_mag_x;
        msg.clipflag_mag_y = status.clipflag_mag_y;
        msg.clipflag_mag_z = status.clipflag_mag_z;
        msg.clipping_indication = status.clipping_indication;
        msg.syncin_marker = status.syncin_marker;
        msg.syncout_marker = status.syncout_marker;
        msg.filter_mode = status.filter_mode; 
        msg.have_gnss_time_pulse = status.have_gnss_time_pulse;
        msg.rtk_status = status.rtk_status; 
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsXsStatusWord)
        {
            xsens_mti_can_ros_driver::XsStatusWord msg;

            XsStatusWord status = packet.status_word;
            assignMessage(msg, status);

            pub.publish(msg);
        }
    }
};

#endif
