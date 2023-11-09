#ifndef PUBLISHER_HELPER_FUNCTION_H
#define PUBLISHER_HELPER_FUNCTION_H

#include <string>
#include <vector>
#include <ros/ros.h>

class PublisherHelperFunctions
{
public:
    PublisherHelperFunctions(/* args */);
    ~PublisherHelperFunctions();

    static void variance_from_stddev_param(std::string param, double *variance_out)
    {
        std::vector<double> stddev;
        if (ros::param::get(param, stddev))
        {
            if (stddev.size() == 3)
            {
                auto squared = [](double x)
                { return x * x; };
                std::transform(stddev.begin(), stddev.end(), variance_out, squared);
            }
            else
            {
                ROS_WARN("Wrong size of param: %s, must be of size 3", param.c_str());
            }
        }
        else
        {
            memset(variance_out, 0, 3 * sizeof(double));
        }
    }
};

PublisherHelperFunctions::PublisherHelperFunctions(/* args */)
{
}

PublisherHelperFunctions::~PublisherHelperFunctions()
{
}

#endif
