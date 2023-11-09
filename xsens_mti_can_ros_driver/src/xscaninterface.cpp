#include "xscaninterface.h"
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnssposepublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/packetcallback.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/statuspublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/utctimepublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/velocitypublisher.h"
// Include other publishers as needed

XsCanInterface::XsCanInterface()
    : socket_(-1)
{
    // Initialize
    m_startframeid = XSENS_SAMPLE_TIME_FRAME_ID; 
    std::string time_option = "mti_utc";
    ros::param::get("~time_option", time_option);
    int temp_startframeid;
    if (ros::param::get("~start_frame_id", temp_startframeid)) {
        m_startframeid = static_cast<uint32_t>(temp_startframeid);
    } 
    ROS_INFO("XsCanINterface::initialize, start_frame_id: %u", m_startframeid);
    ROS_INFO("XsCanINterface::initialize, time_option: %s", time_option.c_str());
	m_timeHandler.setTimeOption(time_option);

}

XsCanInterface::~XsCanInterface()
{
    if (socket_ != -1)
    {
        close(socket_);
    }
}


void XsCanInterface::initialize()
{

    // Initialize SocketCAN socket
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct ifreq ifr;
    //TODO: add to yaml
    strcpy(ifr.ifr_name, "can0"); // Assuming you're using can0 interface
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(socket_, (struct sockaddr *)&addr, sizeof(addr));
}

void XsCanInterface::spinFor()
{
    processCANMessages();
}

void XsCanInterface::registerPublishers(ros::NodeHandle &node)
{
	bool should_publish;

	if (ros::param::get("~pub_imu", should_publish) && should_publish)
	{
		registerCallback(new ImuPublisher(node));
	}
	if (ros::param::get("~pub_quaternion", should_publish) && should_publish)
	{
		registerCallback(new OrientationPublisher(node));
	}
	if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(new AccelerationPublisher(node));
	}
	if (ros::param::get("~pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityPublisher(node));
	}
	if (ros::param::get("~pub_mag", should_publish) && should_publish)
	{
		registerCallback(new MagneticFieldPublisher(node));
	}
	if (ros::param::get("~pub_dq", should_publish) && should_publish)
	{
		registerCallback(new OrientationIncrementsPublisher(node));
	}
	if (ros::param::get("~pub_dv", should_publish) && should_publish)
	{
		registerCallback(new VelocityIncrementPublisher(node));
	}
	if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(new TimeReferencePublisher(node));
	}
	if (ros::param::get("~pub_temperature", should_publish) && should_publish)
	{
		registerCallback(new TemperaturePublisher(node));
	}
	if (ros::param::get("~pub_pressure", should_publish) && should_publish)
	{
		registerCallback(new PressurePublisher(node));
	}
	// if (ros::param::get("~pub_gnss", should_publish) && should_publish)
	// {
	// 	registerCallback(new GnssPublisher(node));
	// }
	if (ros::param::get("~pub_twist", should_publish) && should_publish)
	{
		registerCallback(new TwistPublisher(node));
	}
	if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
	{
		registerCallback(new FreeAccelerationPublisher(node));
	}
	if (ros::param::get("~pub_transform", should_publish) && should_publish)
	{
		registerCallback(new TransformPublisher(node));
	}
	if (ros::param::get("~pub_positionLLA", should_publish) && should_publish)
	{
		registerCallback(new PositionLLAPublisher(node));
	}
	if (ros::param::get("~pub_velocity", should_publish) && should_publish)
	{
		registerCallback(new VelocityPublisher(node));
	}
	if (ros::param::get("~pub_status", should_publish) && should_publish)
	{
		// ROS_INFO("registerCallback StatusPublisher....");
		registerCallback(new StatusPublisher(node));
	}
	if (ros::param::get("~pub_gnsspose", should_publish) && should_publish)
	{
		// ROS_INFO("registerCallback GNSSPOSEPublisher....");
		registerCallback(new GNSSPOSEPublisher(node));
	}
	if (ros::param::get("~pub_utctime", should_publish) && should_publish)
	{
		// ROS_INFO("registerCallback UTCTimePublisher....");
		registerCallback(new UTCTimePublisher(node));
	}
}

void XsCanInterface::processCANMessages()
{
    struct can_frame frame;
    int nbytes = read(socket_, &frame, sizeof(struct can_frame));

    if (nbytes < 0)
    {
        ROS_ERROR("Error while reading CAN frame");
        return;
    }

    if (nbytes < sizeof(struct can_frame))
    {
        ROS_ERROR("Incomplete CAN frame read");
        return;
    }

    uint32_t currentFrameId = frame.can_id;

    if (currentFrameId == m_startframeid)
    {
        // We have reached the start of a new packet, publish the previous packet
        if (lastFrameId != 0xFFFFFFFF) // Check if this is not the first frame ever received
        {
            ros::Time now = m_timeHandler.convertUtcTimeToRosTime(packet);
            for (auto &cb : m_callbacks)
            {
                cb->operator()(packet, now);
            }
        }

        // Clear the packet and start filling it with new data
        packet = XsDataPacket();
        saveToPacket(frame, packet);
    }
    else
    {
        // Still filling the current packet
        saveToPacket(frame, packet);
    }

    // Update the lastFrameId regardless of whether it's a new packet or not
    lastFrameId = currentFrameId;
}


void XsCanInterface::saveToPacket(const can_frame frame, XsDataPacket& packet)
{
    switch (frame.can_id)
        {
        case XSENS_ERROR_FRAME_ID:
            if (xserror_unpack(packet.error, frame))
            {
                packet.containsXsError = true;
            }
            break;
        case XSENS_WARNING_FRAME_ID:
            if (xswarning_unpack(packet.warning, frame))
            {
                packet.containsXsWarning = true;
            }
            break;
        case XSENS_SAMPLE_TIME_FRAME_ID:
            if (xssampletime_unpack(packet.sample_time_fine, frame))
            {
                packet.containsXsSampleTimeFine = true;
                // ROS_INFO("txscaninterface.cpp, get mti_sampletime: %u", packet.sample_time_fine);
            }
            break;

        case XSENS_GROUP_COUNTER_FRAME_ID:
            if (xsgroupcounter_unpack(packet.counter, frame))
            {
                packet.containsXsGroupCounter = true;
            }
            break;

        case XSENS_UTC_FRAME_ID:
            if (xsutctime_unpack(packet.utc_time, frame))
            {
                packet.containsXsUtcTime = true;
                // ROS_INFO("xscaninterface.cpp, get utc time");
            }
            break;

        case XSENS_STATUS_WORD_FRAME_ID:
            if (xsstatusword_unpack(packet.status_word, frame))
            {
                packet.containsXsStatusWord = true;
            }
            break;

        case XSENS_QUATERNION_FRAME_ID:
            if (xsquaternion_unpack(packet.quaternion, frame))
            {
                packet.containsXsQuaternion = true;
            }
            break;

        case XSENS_EULER_ANGLES_FRAME_ID:
            if (xseuler_unpack(packet.euler, frame))
            {
                packet.containsXsEuler = true;
            }
            break;

        case XSENS_DELTA_V_FRAME_ID:
            if (xsdeltavelocity_unpack(packet.delta_velocity, frame))
            {
                packet.containsXsDeltaVelocity = true;
            }
            break;

        case XSENS_RATE_OF_TURN_FRAME_ID:
            if (xsrateofturn_unpack(packet.rate_of_turn, frame))
            {
                packet.containsXsRateOfTurn = true;
            }
            break;

        case XSENS_DELTA_Q_FRAME_ID:
            if (xsquaternion_unpack(packet.delta_q, frame))
            {
                packet.containsXsDeltaQ = true;
            }
            break;

        case XSENS_ACCELERATION_FRAME_ID:
            if (xsacceleration_unpack(packet.acceleration, frame))
            {
                packet.containsXsAcceleration = true;
            }
            break;

        case XSENS_FREE_ACCELERATION_FRAME_ID:
            if (xsacceleration_unpack(packet.free_acceleration, frame))
            {
                packet.containsXsFreeAcceleration = true;
            }
            break;

        case XSENS_MAGNETIC_FIELD_FRAME_ID:
            if (xsmagneticfield_unpack(packet.magnetic_field, frame))
            {
                packet.containsXsMagneticField = true;
            }
            break;

        case XSENS_TEMPERATURE_FRAME_ID:
            if (xstemperature_unpack(packet.temperature, frame))
            {
                packet.containsXsTemperature = true;
            }
            break;

        case XSENS_BAROMETRIC_PRESSURE_FRAME_ID:
            if (xsbaropressure_unpack(packet.baro_pressure, frame))
            {
                packet.containsXsBarometricPressure = true;
            }
            break;

        case XSENS_RATE_OF_TURN_HR_FRAME_ID:
            if (xsrateofturn_unpack(packet.rate_of_turn_hr, frame))
            {
                packet.containsXsRateOfTurnHR = true;
            }
            break;

        case XSENS_ACCELERATION_HR_FRAME_ID:
            if (xsacceleration_unpack(packet.acceleration_hr, frame))
            {
                packet.containsXsAccelerationHR = true;
            }
            break;

        case XSENS_LAT_LON_FRAME_ID:
            if (xslatlon_unpack(packet.lat_lon, frame))
            {
                packet.containsXsLatLon = true;
            }
            break;

        case XSENS_ALTITUDE_ELLIPSOID_FRAME_ID:
            if (xsaltellipsoid_unpack(packet.altitude, frame))
            {
                packet.containsXsAltitudeEllipsoid = true;
            }
            break;

        case XSENS_POSITION_ECEF_X_FRAME_ID:
            if (xspositionecefX_unpack(packet.position_ecef.x, frame))
            {
                packet.containsXsPositionEcefX = true;
            }
            break;

        case XSENS_POSITION_ECEF_Y_FRAME_ID:
            if (xspositionecefX_unpack(packet.position_ecef.y, frame))
            {
                packet.containsXsPositionEcefY = true;
            }
            break;

        case XSENS_POSITION_ECEF_Z_FRAME_ID:
            if (xspositionecefX_unpack(packet.position_ecef.z, frame))
            {
                packet.containsXsPositionEcefZ = true;
            }
            break;

        case XSENS_VELOCITY_FRAME_ID:
            if (xsvelocity_unpack(packet.velocity, frame))
            {
                packet.containsXsVelocity = true;
            }
            break;

        case XSENS_GNSS_RECEIVER_STATUS_FRAME_ID:
            if (xsgnsssreceiverstatus_unpack(packet.gnss_receiver_status, frame))
            {
                packet.containsXsGnssReceiverStatus = true;
            }
            break;

        case XSENS_GNSS_RECEIVER_DOP_FRAME_ID:
            if (xsgnsssreceiverdop_unpack(packet.gnss_receiver_dop, frame))
            {
                packet.containsXsGnssReceiverDop = true;
            }
            break;
        
        //TODO: positionECEF



        default:
            ROS_WARN("Unknown CAN ID");
            break;
        }
}

void XsCanInterface::registerCallback(PacketCallback *cb)
{
    m_callbacks.push_back(cb);

}

void XsCanInterface::closeInterface()
{
    if (socket_ != -1)
    {
        close(socket_);
    }
}
