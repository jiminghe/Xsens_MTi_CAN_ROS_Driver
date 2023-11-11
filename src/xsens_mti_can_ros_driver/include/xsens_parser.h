#ifndef XSENS_PARSER_H
#define XSENS_PARSER_H

#include <iostream>
#include <cstdint>
#include <linux/can.h>

#define XSENS_ERROR_FRAME_ID (0x01u)
#define XSENS_WARNING_FRAME_ID (0x02u)
#define XSENS_SAMPLE_TIME_FRAME_ID (0x05u)
#define XSENS_GROUP_COUNTER_FRAME_ID (0x06u)
#define XSENS_STATUS_WORD_FRAME_ID (0x11u)
#define XSENS_QUATERNION_FRAME_ID (0x21u)
#define XSENS_DELTA_V_FRAME_ID (0x31u)
#define XSENS_RATE_OF_TURN_FRAME_ID (0x32u)
#define XSENS_DELTA_Q_FRAME_ID (0x33u)
#define XSENS_ACCELERATION_FRAME_ID (0x34u)
#define XSENS_FREE_ACCELERATION_FRAME_ID (0x35u)
#define XSENS_RATE_OF_TURN_HR_FRAME_ID (0x61u)
#define XSENS_ACCELERATION_HR_FRAME_ID (0x62u)
#define XSENS_MAGNETIC_FIELD_FRAME_ID (0x41u)
#define XSENS_TEMPERATURE_FRAME_ID (0x51u)
#define XSENS_BAROMETRIC_PRESSURE_FRAME_ID (0x52u)
#define XSENS_UTC_FRAME_ID (0x07u)
#define XSENS_EULER_ANGLES_FRAME_ID (0x22u)
#define XSENS_LAT_LON_FRAME_ID (0x71u)
#define XSENS_ALTITUDE_ELLIPSOID_FRAME_ID (0x72u)
#define XSENS_POSITION_ECEF_X_FRAME_ID (0x73u)
#define XSENS_POSITION_ECEF_Y_FRAME_ID (0x74u)
#define XSENS_POSITION_ECEF_Z_FRAME_ID (0x75u)
#define XSENS_VELOCITY_FRAME_ID (0x76u)
#define XSENS_GNSS_RECEIVER_STATUS_FRAME_ID (0x79u)
#define XSENS_GNSS_RECEIVER_DOP_FRAME_ID (0x7au)



struct XsError {
    bool CEI_OutputBufferOverflow = false; // The output buffer is full, at least one message was dropped
};

struct XsWarning {
    uint8_t warning_code = 0;
};


struct XsSampleTimeFine {
    uint32_t timestamp = 0;
};


struct XsGroupCounter {
    uint16_t counter = 0;
};


struct XsUtcTime
{
    //the raw CAN data gives uint8 year like 23, but we need to show it like 2023
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t tenthms = 0;
};

struct XsQuaternion
{
    float q0 = 1;
    float q1 = 0;
    float q2 = 0;
    float q3 = 0;
};

struct XsEuler
{
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
};

struct XsAcceleration
{
    float x = 0;
    float y = 0;
    float z = 0;
};

struct XsFreeAcceleration: public XsAcceleration
{
};

struct XsAccelerationHR: public XsAcceleration
{
};

struct XsRateOfTurn
{
    float x = 0;
    float y = 0;
    float z = 0;
};

struct XsRateOfTurnHR: public XsRateOfTurn
{
};

struct XsDeltaVelocity {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct XsDeltaQ: public XsQuaternion {
};

struct XsMagneticField
{
    float x = 0;
    float y = 0;
    float z = 0;
};

struct XsLatLon {
    double latitude = 0;
    double longitude = 0;
};


struct XsAltitudeEllipsoid {
    double alt_ellipsoid = 0;
};

struct XsPositionEcef {
    double x = 0;
    double y = 0;
    double z = 0;
};

struct XsVelocity {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct XsTemperature {
    float temperature = 0;
};

struct XsBarometricPressure {
    uint32_t pressure = 0;
};


struct XsGnssReceiverStatus {
    uint8_t fix_type = 0;
    uint8_t num_sv = 0;
    uint8_t flags = 0;
    uint8_t valid = 0;
    uint8_t num_svs = 0;
};


struct XsGnssReceiverDop {
    float pdop = 0;
    float tdop = 0;
    float vdop = 0;
    float hdop = 0;
};


struct XsStatusWord {
    bool selftest = false;
    bool filter_valid = false;
    bool gnss_fix = false;
    uint8_t no_rotation_update_status = 0; //0: not running, 2: aborted, 3:running
    bool representative_motion = false;
    bool clock_bias_estimation = false;
    bool clipflag_acc_x = false;
    bool clipflag_acc_y = false;
    bool clipflag_acc_z = false;
    bool clipflag_gyr_x = false;
    bool clipflag_gyr_y = false;
    bool clipflag_gyr_z = false;
    bool clipflag_mag_x = false;
    bool clipflag_mag_y = false;
    bool clipflag_mag_z = false;
    bool clipping_indication = false;
    bool syncin_marker = false;
    bool syncout_marker = false;
    uint8_t filter_mode = 0; //0: Without GNSS, 1: Coasting, 3: With GNSS
    bool have_gnss_time_pulse = false;
    uint8_t rtk_status = 0; //0: No RTK, 1: RTK Floating, 2: RTK Fix
};

struct XsDataPacket{
    XsError error;
    XsWarning warning;
    XsSampleTimeFine sample_time_fine;
    XsGroupCounter counter;
    XsUtcTime utc_time;
    XsStatusWord status_word;
    XsQuaternion quaternion;
    XsEuler euler;
    XsDeltaVelocity delta_velocity;
    XsRateOfTurn rate_of_turn;
    XsDeltaQ delta_q;
    XsAcceleration acceleration;
    XsFreeAcceleration free_acceleration;
    XsMagneticField magnetic_field;
    XsTemperature temperature;
    XsBarometricPressure baro_pressure;
    XsAccelerationHR acceleration_hr;
    XsRateOfTurnHR rate_of_turn_hr;
    XsLatLon lat_lon;
    XsAltitudeEllipsoid altitude;
    XsPositionEcef position_ecef;
    XsVelocity velocity;
    XsGnssReceiverStatus gnss_receiver_status;
    XsGnssReceiverDop gnss_receiver_dop;


    bool containsXsError = false;
    bool containsXsWarning = false;
    bool containsXsSampleTimeFine = false;
    bool containsXsGroupCounter = false;
    bool containsXsUtcTime = false;
    bool containsXsStatusWord = false;
    bool containsXsQuaternion = false;
    bool containsXsEuler = false;
    bool containsXsDeltaVelocity = false;
    bool containsXsRateOfTurn = false;
    bool containsXsDeltaQ = false;
    bool containsXsAcceleration = false;
    bool containsXsFreeAcceleration = false;
    bool containsXsMagneticField = false;
    bool containsXsTemperature = false;
    bool containsXsBarometricPressure = false;
    bool containsXsRateOfTurnHR = false;
    bool containsXsAccelerationHR = false;
    bool containsXsLatLon = false;
    bool containsXsAltitudeEllipsoid = false;
    bool containsXsPositionEcefX = false;
    bool containsXsPositionEcefY = false;
    bool containsXsPositionEcefZ = false;
    bool containsXsVelocity = false;
    bool containsXsGnssReceiverStatus = false;
    bool containsXsGnssReceiverDop = false;
};


bool xserror_unpack(XsError &e, const struct can_frame &frame);
bool xswarning_unpack(XsWarning &w, const struct can_frame &frame);
bool xssampletime_unpack(XsSampleTimeFine &timestamp, const struct can_frame &frame);
bool xsgroupcounter_unpack(XsGroupCounter &counter, const struct can_frame &frame);
bool xsutctime_unpack(XsUtcTime &utctime, const struct can_frame &frame);
//can be used to parse quaternion, and deltaQ
bool xsquaternion_unpack(XsQuaternion &q, const struct can_frame &frame);
bool xseuler_unpack(XsEuler &euler, const struct can_frame &frame);
//can be used to parse acc, free_acc, and AccelerationHR
bool xsacceleration_unpack(XsAcceleration &acc, const struct can_frame &frame);
//can be used to RateOfTurn and RateOfTurnHR
bool xsrateofturn_unpack(XsRateOfTurn &gyro, const struct can_frame &frame);
bool xsdeltavelocity_unpack(XsDeltaVelocity &dv, const struct can_frame &frame);
bool xsmagneticfield_unpack(XsMagneticField &mag, const struct can_frame &frame);
bool xslatlon_unpack(XsLatLon &latlon, const struct can_frame &frame);
bool xsaltellipsoid_unpack(XsAltitudeEllipsoid &alt, const struct can_frame &frame);
bool xspositionecefX_unpack(double &pos, const struct can_frame &frame);
bool xsvelocity_unpack(XsVelocity &vel,  const struct can_frame &frame);
bool xsstatusword_unpack(XsStatusWord &status, const struct can_frame &frame);
bool xstemperature_unpack(XsTemperature &temp, const struct can_frame &frame);
bool xsbaropressure_unpack(XsBarometricPressure &temp, const struct can_frame &frame);
bool xsgnsssreceiverstatus_unpack(XsGnssReceiverStatus &gnssStatus, const struct can_frame &frame);
bool xsgnsssreceiverdop_unpack(XsGnssReceiverDop &gnssDop, const struct can_frame &frame);


#endif