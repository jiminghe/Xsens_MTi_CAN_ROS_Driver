#include "xsens_time_handler.h"

XsensTimeHandler::XsensTimeHandler() : time_option(""), prevSampleTimeFine(0), isFirstFrame(true) {}

ros::Time XsensTimeHandler::convertUtcTimeToRosTime(const XsDataPacket &packet)
{
    std::lock_guard<std::mutex> lock(m_mutex);  // Lock the mutex at the beginning of the method
    if (time_option == "mti_utc" && packet.containsXsUtcTime)
    {
        //ROS_INFO("time_option is mti_utc");
        struct tm timeinfo = {0};

        timeinfo.tm_year = packet.utc_time.year - 1900;
        timeinfo.tm_mon = packet.utc_time.month - 1;
        timeinfo.tm_mday = packet.utc_time.day;
        timeinfo.tm_hour = packet.utc_time.hour;
        timeinfo.tm_min = packet.utc_time.minute;
        timeinfo.tm_sec = packet.utc_time.second;

        time_t epochSeconds = timegm(&timeinfo);

        return ros::Time(epochSeconds, 100*packet.utc_time.tenthms);
    }
    else if (time_option == "mti_sampletime" && packet.containsXsSampleTimeFine)
    {
        uint32_t currentSampleTimeFine = packet.sample_time_fine.timestamp;

        if (isFirstFrame)
        {
            //ROS_INFO("time_option is mti_sampletime, first frame, timestamp: %u", currentSampleTimeFine);
            isFirstFrame = false;
            firstUTCTimestamp = ros::Time::now();
            prevSampleTimeFine = currentSampleTimeFine;
            return firstUTCTimestamp;
        }
        else
        {
            int64_t timeDiff = static_cast<int64_t>(currentSampleTimeFine) - static_cast<int64_t>(prevSampleTimeFine);

            // Checking for wraparound
            if (timeDiff < 0)
            {
                // Check if the difference is significant enough to be considered a wraparound
                if (timeDiff < -static_cast<int64_t>(m_RollOver / 2))
                {
                    // If wrap around occurred, adjust timeDiff
                    //ROS_INFO("time_option is mti_sampletime, Wraparound Detected. Current: %u, Previous: %u", currentSampleTimeFine, prevSampleTimeFine);
                    timeDiff += m_RollOver;
                }
                // else
                // {
                //     //// No adjustment on the cases when the packet comes later with a smaller sampleTimeFine.
                //     ROS_WARN("Minor timestamp decrement detected but not considered as wraparound. Current: %u, Previous: %u", currentSampleTimeFine, prevSampleTimeFine);
                // }
            }

            ros::Duration deltaTime(timeDiff * 0.0001); // Convert to seconds using the multiplier 0.0001
            firstUTCTimestamp += deltaTime;

            prevSampleTimeFine = currentSampleTimeFine;
            return firstUTCTimestamp;
        }
    }
    else
    {
        // ROS_INFO("time_option is host controller time");
        return ros::Time::now(); // returns ros time
    }
}

void XsensTimeHandler::setTimeOption(const std::string &option)
{
    std::lock_guard<std::mutex> lock(m_mutex); // Lock the mutex
    time_option = option;
}

void XsensTimeHandler::setRollover(const uint32_t &rollOver)
{
    std::lock_guard<std::mutex> lock(m_mutex); // Lock the mutex
    m_RollOver = rollOver;
}
