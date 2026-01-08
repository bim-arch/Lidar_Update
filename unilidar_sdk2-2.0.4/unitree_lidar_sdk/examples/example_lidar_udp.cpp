/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "example.h"

int main(int argc, char *argv[])
{

    // Initialize
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.2";

    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip))
    {
        printf("Unilidar initialization failed! Exit here!\n");
        exit(-1);
    }
    else
    {
        printf("Unilidar initialization succeed!\n");
    }

    // First set the work mode to manual start (bit 4 = 1)
    uint32_t workMode = 16;  // Binary: 00010000 (bit 4 = 1, others = 0)
    std::cout << "Setting LiDAR work mode to: " << workMode << " (manual start mode)" << std::endl;
    lreader->setLidarWorkMode(workMode);
    sleep(1);  // Give it time to process

    // Now explicitly start the rotation
    std::cout << "Sending start rotation command..." << std::endl;
    lreader->startLidarRotation();
    sleep(1);  // Give it time to start

    // Process
    exampleProcess(lreader);
    
    return 0;
}