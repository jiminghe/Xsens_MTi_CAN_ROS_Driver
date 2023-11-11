#include <ros/ros.h>
#include "xscaninterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xsens_mti_can_ros_driver");
    ros::NodeHandle node;

    // Create the CAN interface and initialize it
    XsCanInterface *canInterface = new XsCanInterface();
    canInterface->initialize();
    canInterface->registerPublishers(node);


    // Start processing messages
    while (ros::ok())
    {
        canInterface->spinFor();
        ros::spinOnce();
    }

    canInterface->closeInterface();

    return 0;
}