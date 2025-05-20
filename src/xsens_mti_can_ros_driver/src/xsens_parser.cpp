#include "xsens_parser.h"

bool xserror_unpack(XsError &e, const struct can_frame &frame)
{
    if (frame.can_dlc < 1)
    {
        std::cerr << "XsError frame DLC must be at least 1 byte" << std::endl;
        return false;
    }

    e.CEI_OutputBufferOverflow = (frame.data[0] == 0x01);

    return true;
}

bool xswarning_unpack(XsWarning &w, const struct can_frame &frame)
{
    if (frame.can_dlc < 1)
    {
        std::cerr << "XsWarning frame DLC must be at least 1 byte" << std::endl;
        return false;
    }

    w.warning_code = frame.data[0];

    return true;
}

bool xssampletime_unpack(XsSampleTimeFine &sampleTime, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        std::cerr << "XsSampleTimeFine frame DLC must be at least 4 bytes" << std::endl;
        return false;
    }

    // Unpack and assemble the timestamp
    sampleTime.timestamp = 0; // initialize to 0
    sampleTime.timestamp |= static_cast<uint32_t>(frame.data[0]) << 24;
    sampleTime.timestamp |= static_cast<uint32_t>(frame.data[1]) << 16;
    sampleTime.timestamp |= static_cast<uint32_t>(frame.data[2]) << 8;
    sampleTime.timestamp |= static_cast<uint32_t>(frame.data[3]);

    return true;
}

bool xsgroupcounter_unpack(XsGroupCounter &groupCounter, const struct can_frame &frame)
{
    if (frame.can_dlc < 2)
    {
        std::cerr << "XsGroupCounter frame DLC must be at least 2 bytes" << std::endl;
        return false;
    }

    groupCounter.counter = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];

    return true;
}

bool xsutctime_unpack(XsUtcTime &utctime, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        std::cerr << "XsUtcTime frame DLC must be at least 8 bytes" << std::endl;
        return false;
    }
    uint8_t year = frame.data[0];
    //we need to convert the uint8_t to the full year, for example 23 to 2023
    //this code is valid till the year of 2100
    if (year < 70)
    {
        utctime.year = 2000 + year;
    }
    else
    {
        utctime.year = 1900 + year;
    }
    utctime.month = frame.data[1];
    utctime.day = frame.data[2];
    utctime.hour = frame.data[3];
    utctime.minute = frame.data[4];
    utctime.second = frame.data[5];
    utctime.tenthms = static_cast<uint16_t>((frame.data[6] << 8) | frame.data[7]);

    return true;
}

bool xsquaternion_unpack(XsQuaternion &q, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        std::cerr << "XsQuaternion frame DLC must be at least 8 bytes" << std::endl;
        return false;
    }

    double scale = 1.0 / ((1 << 15) - 1);
    float *q_arr[] = {&q.q0, &q.q1, &q.q2, &q.q3}; // Array of pointers to quaternion components

    for (size_t i = 0; i < 4; ++i)
    {
        // Combine two bytes to make a 16-bit signed integer
        int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
        // Scale the value and store it in the quaternion
        *q_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xseuler_unpack(XsEuler &euler, const struct can_frame &frame)
{

    if (frame.can_dlc < 6)
    {
        std::cerr << "XsEuler frame DLC must be at least 6 bytes" << std::endl;
        return false;
    }

    double scale = 1.0 / (1 << 7); // 0.0078125
    float *euler_arr[] = {&euler.roll, &euler.pitch, &euler.yaw};

    for (size_t i = 0; i < 3; ++i)
    {
        int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
        *euler_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xsacceleration_unpack(XsAcceleration &acc, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        std::cerr << "XsAcceleration frame DLC must be at least 6 bytes" << std::endl;
        return false;
    }

    double scale = 1.0 / (1 << 8); // 0.00390625
    float *acc_arr[] = {&acc.x, &acc.y, &acc.z};

    for (size_t i = 0; i < 3; ++i)
    {
        int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
        *acc_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xsrateofturn_unpack(XsRateOfTurn &gyro, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        std::cerr << "XsRateOfTurn frame DLC must be at least 6 bytes" << std::endl;
        return false;
    }

    double scale = 1.0 / (1 << 9); // 0.001953125
    float *gyro_arr[] = {&gyro.x, &gyro.y, &gyro.z};

    for (size_t i = 0; i < 3; ++i)
    {
        int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
        *gyro_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xsdeltavelocity_unpack(XsDeltaVelocity &dv, const struct can_frame &frame) {
    if (frame.can_dlc < 7) {  // Check if Data Length Code (DLC) is at least 7 bytes
        std::cerr << "XsDeltaVelocity frame DLC must be at least 7 bytes" << std::endl;
        return false;
    }

    uint8_t exponent = frame.data[6];
    double scale = 1.0 / (1 << exponent) ;
    float *dv_arr[] = {&dv.x, &dv.y, &dv.z}; 

    for (size_t i = 0; i < 3; ++i)
    {
        // Combine two bytes to make a 16-bit signed integer
        int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
        // Scale the value and store it in the dv
        *dv_arr[i] = static_cast<float>(value * scale);
    }


    return true;
}


bool xsmagneticfield_unpack(XsMagneticField &mag, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        std::cerr << "XsMagneticField frame DLC must be at least 6 bytes" << std::endl;
        return false;
    }

    double scale = 1.0 / (1 << 10); // 0.0009765625
    float *mag_arr[] = {&mag.x, &mag.y, &mag.z};

    for (size_t i = 0; i < 3; ++i)
    {
        int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
        *mag_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xslatlon_unpack(XsLatLon &latlon, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        std::cerr << "XsLatLon frame DLC must be at least 8 bytes" << std::endl;
        return false;
    }
    
    int32_t latitude = 0;
    int32_t longitude = 0;
    double scale_lat = 1.0 / (1 << 24); // 5.9604644775e-08
    double scale_lon = 1.0 / (1 << 23); // 1.1920928955e-07
    
    // Unpack and assemble latitude (using int32_t for proper sign handling)
    latitude |= static_cast<int32_t>(frame.data[0]) << 24;
    latitude |= static_cast<int32_t>(frame.data[1]) << 16;
    latitude |= static_cast<int32_t>(frame.data[2]) << 8;
    latitude |= static_cast<int32_t>(frame.data[3]);
    
    // Unpack and assemble longitude (using int32_t for proper sign handling)
    longitude |= static_cast<int32_t>(frame.data[4]) << 24;
    longitude |= static_cast<int32_t>(frame.data[5]) << 16;
    longitude |= static_cast<int32_t>(frame.data[6]) << 8;
    longitude |= static_cast<int32_t>(frame.data[7]);
    
    // Convert to double
    latlon.latitude = static_cast<double>(latitude) * scale_lat;
    latlon.longitude = static_cast<double>(longitude) * scale_lon;
    
    return true;
}

bool xsaltellipsoid_unpack(XsAltitudeEllipsoid &alt, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        std::cerr << "XsAltitudeEllipsoid frame DLC must be at least 4 bytes" << std::endl;
        return false;
    }

    int32_t alt_ellipsoid = 0;
    double scale = 1.0 / (1 << 15); // 3.0517578125e-05

    // Unpack and assemble alt_ellipsoid
    alt_ellipsoid |= static_cast<int32_t>(frame.data[0]) << 24;
    alt_ellipsoid |= static_cast<int32_t>(frame.data[1]) << 16;
    alt_ellipsoid |= static_cast<int32_t>(frame.data[2]) << 8;
    alt_ellipsoid |= static_cast<int32_t>(frame.data[3]);

    alt.alt_ellipsoid = static_cast<double>(alt_ellipsoid) * scale;
    return true;
}

bool xspositionecefX_unpack(double &pos, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)  // Only need 4 bytes for X component
    {
        std::cerr << "XsPositionEcef_X frame DLC must be at least 4 bytes" << std::endl;
        return false;
    }
    
    int32_t x = 0;
    double scale = 1.0 / (1 << 8); // 0.00390625
    
    // Unpack and assemble x (using int32_t for proper sign handling)
    x |= static_cast<int32_t>(frame.data[0]) << 24;
    x |= static_cast<int32_t>(frame.data[1]) << 16;
    x |= static_cast<int32_t>(frame.data[2]) << 8;
    x |= static_cast<int32_t>(frame.data[3]);
    
    // Convert to double (cast first, then multiply)
    pos = static_cast<double>(x) * scale;
    
    return true;
}

bool xsvelocity_unpack(XsVelocity &vel, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        std::cerr << "XsVelocity frame DLC must be at least 6 bytes" << std::endl;
        return false;
    }
    
    double scale = 1.0 / (1 << 6); // 0.015625
    
    // Process each component (x, y, z)
    // First parse the bytes into int16_t values (big endian)
    int16_t velX = static_cast<int16_t>((frame.data[0] << 8) | frame.data[1]);
    int16_t velY = static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
    int16_t velZ = static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]);
    
    // Convert to float with scaling
    vel.x = static_cast<float>(velX) * scale;
    vel.y = static_cast<float>(velY) * scale;
    vel.z = static_cast<float>(velZ) * scale;
    
    return true;
}

bool xsstatusword_unpack(XsStatusWord &status, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        std::cerr << "XsStatusWord frame DLC must be at least 4 bytes" << std::endl;
        return false;
    }

    // packing 4 bytes into a single 32-bit unsigned integer variable
    // big endian, most significant byte (MSB) is placed in the highest memory address 
    uint32_t statusWord = (frame.data[0]<<24) | (frame.data[1]<<16) | (frame.data[2]<<8) | (frame.data[3]);


    status.selftest = statusWord & (1 << 0);
    status.filter_valid = statusWord & (1 << 1);
    status.gnss_fix = statusWord & (1 << 2);
    status.no_rotation_update_status = (statusWord >> 3) & 0x3; // status & 0x18;
    status.representative_motion = statusWord & (1 << 5);
    status.clock_bias_estimation = statusWord & (1 << 6);
    status.clipflag_acc_x = statusWord & (1 << 8);
    status.clipflag_acc_y = statusWord & (1 << 9);
    status.clipflag_acc_z = statusWord & (1 << 10);
    status.clipflag_gyr_x = statusWord & (1 << 11);
    status.clipflag_gyr_y = statusWord & (1 << 12);
    status.clipflag_gyr_z = statusWord & (1 << 13);
    status.clipflag_mag_x = statusWord & (1 << 14);
    status.clipflag_mag_y = statusWord & (1 << 15);
    status.clipflag_mag_z = statusWord & (1 << 16);
    status.clipping_indication = statusWord & (1 << 19);
    status.syncin_marker = statusWord & (1 << 21);
    status.syncout_marker = statusWord & (1 << 22);
    status.filter_mode = (statusWord >> 23) & 0x7; // status & 0x03800000;
    status.have_gnss_time_pulse = statusWord & (1 << 26);
    status.rtk_status = (statusWord >> 27) & 0x3; // status & 0x18000000;

    return true;
}


bool xstemperature_unpack(XsTemperature &temp, const struct can_frame &frame) {
    if (frame.can_dlc < 2) {
        std::cerr << "XsTemperature frame DLC must be at least 2 bytes for temperature" << std::endl;
        return false;
    }

    uint16_t temperature = 0;
    double scale = 1.0 / (1 << 8);

    temperature |= static_cast<uint16_t>(frame.data[0]) << 8;  // MSB
    temperature |= static_cast<uint16_t>(frame.data[1]);       // LSB

    temp.temperature = static_cast<float>(temperature * scale);

    return true;
}


bool xsbaropressure_unpack(XsBarometricPressure &pressure, const struct can_frame &frame) {
    if (frame.can_dlc < 4) {
        std::cerr << "XsBarometricPressure frame DLC must be at least 4 bytes for barometric pressure" << std::endl;
        return false;
    }

    uint32_t temp_pressure = 0;
    double scale = 1.0 / (1 << 15);

    temp_pressure |= static_cast<uint32_t>(frame.data[0]) << 24;  // MSB
    temp_pressure |= static_cast<uint32_t>(frame.data[1]) << 16;
    temp_pressure |= static_cast<uint32_t>(frame.data[2]) << 8;
    temp_pressure |= static_cast<uint32_t>(frame.data[3]);       // LSB

    pressure.pressure = static_cast<float>(temp_pressure * scale);

    return true;
}


bool xsgnsssreceiverstatus_unpack(XsGnssReceiverStatus &gnssStatus, const struct can_frame &frame)
{
    if(frame.can_dlc < 5)
    {
        std::cerr << "XsGnssReceiverStatus frame DLC must be at least 5 bytes" << std::endl;
        return false;
    }

    gnssStatus.fix_type = frame.data[0];
    gnssStatus.num_sv = frame.data[1];
    gnssStatus.flags = frame.data[2];
    gnssStatus.valid = frame.data[3];
    gnssStatus.num_svs = frame.data[4];

    return true;
}



bool xsgnsssreceiverdop_unpack(XsGnssReceiverDop &gnssDop, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        std::cerr << "XsGnssReceiverDop frame DLC must be at least 8 bytes" << std::endl;
        return false;
    }

    // Scaling factor
    const double scale = 0.01;

    // Unpack the 8 bytes into the XsGnssReceiverDop structure
    uint16_t pdop = static_cast<uint16_t>((frame.data[0] << 8) | frame.data[1]);
    uint16_t tdop = static_cast<uint16_t>((frame.data[2] << 8) | frame.data[3]);
    uint16_t vdop = static_cast<uint16_t>((frame.data[4] << 8) | frame.data[5]);
    uint16_t hdop = static_cast<uint16_t>((frame.data[6] << 8) | frame.data[7]);

    // Apply scaling
    gnssDop.pdop = static_cast<float>(pdop * scale);
    gnssDop.tdop = static_cast<float>(tdop * scale);
    gnssDop.vdop = static_cast<float>(vdop * scale);
    gnssDop.hdop = static_cast<float>(hdop * scale);

    return true;
}




