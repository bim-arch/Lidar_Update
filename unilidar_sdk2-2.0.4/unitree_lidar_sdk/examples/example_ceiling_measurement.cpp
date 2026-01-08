/**********************************************************************
 * Ceiling Measurement Example for Unitree LiDAR L2 - IMPROVED
 * 
 * Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
 * 
 * IMPORTANT: Make sure LiDAR is pointing STRAIGHT UP at the ceiling!
 * 
 * Place this file in: unitree_lidar_sdk/examples/example_ceiling_measurement.cpp
 ***********************************************************************/

#include "unitree_lidar_sdk.h"
#include "ceiling_corner_detector.h"
#include <unistd.h>
#include <chrono>

using namespace unitree_lidar_sdk;

// Global pointer for cleanup
UnitreeLidarReader* g_lreader = nullptr;

void cleanup() {
    if (g_lreader != nullptr) {
        std::cout << "\nStopping LiDAR rotation...\n";
        g_lreader->stopLidarRotation();
        std::cout << "Cleanup complete.\n";
    }
}

int main(int argc, char *argv[])
{
    installSignalHandler();
    
    std::cout << "============================================\n";
    std::cout << "  UNITREE LIDAR L2 - CEILING MEASUREMENT\n";
    std::cout << "============================================\n";
    std::cout << "\n";
    std::cout << "IMPORTANT: Make sure LiDAR is pointing STRAIGHT UP!\n";
    std::cout << "Press Ctrl+C anytime to stop.\n\n";
    
    // ================================================================
    // STEP 1: Initialize LiDAR
    // ================================================================
    
    UnitreeLidarReader *lreader = createUnitreeLidarReader();
    g_lreader = lreader;

    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 4000000;

    std::cout << "Initializing LiDAR on " << port << "...\n";
    
    if (lreader->initializeSerial(port, baudrate))
    {
        std::cout << "ERROR: Unilidar initialization failed!\n";
        std::cout << "Check:\n";
        std::cout << "  1. ls /dev/ttyACM*\n";
        std::cout << "  2. sudo chmod 666 /dev/ttyACM0\n";
        std::cout << "  3. usbipd attach --wsl --busid X-X\n";
        return -1;
    }
    
    std::cout << "Unilidar initialization succeed!\n";

    // ================================================================
    // STEP 2: Configure LiDAR
    // ================================================================
    
    if (!isRunning()) { cleanup(); return 0; }
    
    // Follow the serial sample: start the motor first, then switch to serial/3D mode (auto start)
    std::cout << "Sending start rotation command..." << std::endl;
    lreader->startLidarRotation();
    sleep(2);

    uint32_t workMode = 8;  // Binary 00001000: serial connection, auto start, 3D mode
    std::cout << "Setting LiDAR work mode to: " << workMode
              << " (serial mode, auto start)" << std::endl;
    lreader->setLidarWorkMode(workMode);
    sleep(2);

    // Allow extra time for the laser to stabilise before we expect cloud packets
    std::cout << "Waiting for LiDAR to initialize (8 seconds)..." << std::endl;
    sleep(8);
    lreader->clearBuffer();
    
    // Debug: Check for point cloud data
    std::cout << "\nChecking for point cloud data (this may take up to 30 seconds)..." << std::endl;
    int attempts = 0;
    bool gotPointCloud = false;

    int pointCloudPackets = 0;
    int ackPackets = 0;
    int imuPackets = 0;
    int otherPackets = 0;
    int lastPacketType = 0;
    bool retried = false;
    
    // Wait up to 30 seconds for point cloud data (60 attempts × 500ms = 30 seconds)
    while (isRunning() && attempts < 60) {
        auto sliceStart = std::chrono::steady_clock::now();
        auto sliceEnd = sliceStart + std::chrono::milliseconds(500);

        while (isRunning() && !gotPointCloud && std::chrono::steady_clock::now() < sliceEnd) {
            int packetType = lreader->runParse();

            if (packetType == 0) {
                usleep(1000);
                continue;
            }

            if (packetType == 102) {  // 102 is point cloud data
                pointCloudPackets++;
                if (!gotPointCloud) {
                    gotPointCloud = true;
                    std::cout << "\nSuccess! Receiving point cloud data (type 102)" << std::endl;
                }
                break;
            } else if (packetType > 0) {
                if (packetType != lastPacketType) {
                    std::cout << "\nReceived packet type: " << packetType << std::endl;
                }
                lastPacketType = packetType;
                if (packetType == 101) {
                    ackPackets++;
                } else if (packetType == 104) {
                    imuPackets++;
                } else {
                    otherPackets++;
                }
            }
        }

        if (gotPointCloud || !isRunning()) {
            break;
        }

        attempts++;
        
        // Show progress every 5 seconds (every 10 attempts × 500ms)
        if (attempts % 10 == 0) {
            if (gotPointCloud) {
                std::cout << "Received " << pointCloudPackets << " point cloud packets so far... ("
                          << (attempts / 2) << " seconds elapsed)" << std::endl;
            } else {
                std::cout << "Waiting for point cloud data... (" << (attempts / 2)
                          << " seconds elapsed)" << std::endl;
            }
        }

        if (!gotPointCloud && !retried && attempts == 20 && imuPackets > 0) {
            std::cout << "\nRetrying LiDAR work mode / rotation..." << std::endl;
            lreader->clearBuffer();
            lreader->setLidarWorkMode(workMode);
            sleep(1);
            lreader->startLidarRotation();
            sleep(1);
            lreader->clearBuffer();
            retried = true;
        }
    }
    
    if (!gotPointCloud) {
        if (!isRunning()) {
            cleanup();
            return 0;
        }
        std::cout << "\nWARNING: No point cloud data received after " << (attempts / 2) << " seconds!" << std::endl;
        std::cout << "Packets received: ack=" << ackPackets
                  << ", imu=" << imuPackets
                  << ", other=" << otherPackets
                  << ", point=" << pointCloudPackets
                  << std::endl;
        if (lastPacketType > 0) {
            std::cout << "Last received packet type: " << lastPacketType << std::endl;
        }
        if (retried) {
            std::cout << "Retry attempted: yes" << std::endl;
        }
        std::cout << "Please check:" << std::endl;
        std::cout << "1. Is the LiDAR physically spinning? (should rotate when powered on)" << std::endl;
        std::cout << "2. Is it pointing straight up at the ceiling?" << std::endl;
        std::cout << "3. Try power cycling the LiDAR and running again" << std::endl;
        cleanup();
        return -1;
    }
    
    std::cout << "LiDAR is ready for measurement!" << std::endl;
    
    if (!isRunning()) { cleanup(); return 0; }
    
    // Get version info
    std::string versionHardware, versionFirmware, versionSDK;
    while (!lreader->getVersionOfLidarFirmware(versionFirmware) && isRunning()) {
        lreader->runParse();
    }
    
    if (isRunning()) {
        lreader->getVersionOfLidarHardware(versionHardware);
        lreader->getVersionOfSDK(versionSDK);
        std::cout << "Hardware version: " << versionHardware << "\n";
        std::cout << "Firmware version: " << versionFirmware << "\n";
        std::cout << "SDK version: " << versionSDK << "\n";
    }
    
    // ================================================================
    // STEP 3: Configure Detection Parameters
    // ================================================================
    
    if (!isRunning()) { cleanup(); return 0; }
    
    CeilingDetectorConfig config;
    
    // ============================================================
    // ADJUST THESE VALUES FOR YOUR ROOM!
    // ============================================================
    
    config.collection_time_seconds = 5.0;   // Scan time (increase for better accuracy)
    
    // CEILING HEIGHT RANGE (in meters)
    // Set these based on your room!
    // Typical rooms: 2.4m - 3.0m ceiling
    // High ceilings: 3.0m - 4.5m
    config.min_ceiling_height_m = 2.0;      // Minimum expected ceiling height
    config.max_ceiling_height_m = 4.0;      // Maximum expected ceiling height
    
    config.ceiling_thickness_m = 0.25;      // Detection tolerance (25cm)
    config.percentile_for_ceiling = 0.95;   // Use 95th percentile (ignores outliers)
    
    config.ransac_iterations = 150;
    config.ransac_threshold_m = 0.03;
    config.verbose = true;
    
    // ============================================================
    
    std::cout << "\n--- Configuration ---\n";
    std::cout << "Expected ceiling height: " << config.min_ceiling_height_m 
              << " - " << config.max_ceiling_height_m << " meters\n";
    std::cout << "Collection time: " << config.collection_time_seconds << " seconds\n";
    
    // ================================================================
    // STEP 4: Run Detection
    // ================================================================
    
    CeilingCornerDetector detector(config);
    CeilingCorners result = detector.detect(lreader);
    
    // ================================================================
    // STEP 5: Final Output
    // ================================================================
    
    if (result.valid) {
        std::cout << "\n";
        std::cout << "+============================================+\n";
        std::cout << "|         FINAL MEASUREMENTS                 |\n";
        std::cout << "+============================================+\n";
        std::cout << std::fixed;
        std::cout << "| Ceiling Height: " << std::setw(8) << std::setprecision(1) 
                  << result.ceiling_height_mm << " mm (" 
                  << std::setprecision(2) << result.ceiling_height_mm/1000 << " m)  |\n";
        std::cout << std::setprecision(1);
        std::cout << "| Length:         " << std::setw(8) << result.length_mm << " mm (" 
                  << std::setprecision(2) << result.length_mm/1000 << " m)  |\n";
        std::cout << std::setprecision(1);
        std::cout << "| Width:          " << std::setw(8) << result.width_mm << " mm (" 
                  << std::setprecision(2) << result.width_mm/1000 << " m)  |\n";
        std::cout << "| Diagonal 1:     " << std::setw(8) << std::setprecision(1) 
                  << result.diagonal1_mm << " mm           |\n";
        std::cout << "| Diagonal 2:     " << std::setw(8) << result.diagonal2_mm << " mm           |\n";
        std::cout << "| Area:           " << std::setw(8) << std::setprecision(2) 
                  << result.area_sqm << " sq.m         |\n";
        std::cout << "+============================================+\n";
        
        std::cout << "\nCorner Coordinates (X, Y, Z in mm):\n";
        std::cout << std::setprecision(1);
        for (int i = 0; i < 4; i++) {
            std::cout << "  C" << (i+1) << ": (" 
                      << std::setw(8) << result.corners[i].x * 1000 << ", "
                      << std::setw(8) << result.corners[i].y * 1000 << ", "
                      << std::setw(8) << result.corners[i].z * 1000 << ")\n";
        }
    } else {
        std::cout << "\n";
        std::cout << "+============================================+\n";
        std::cout << "|         MEASUREMENT FAILED                 |\n";
        std::cout << "+============================================+\n";
        std::cout << "| Check that LiDAR is pointing UP at ceiling |\n";
        std::cout << "| Adjust min/max ceiling height if needed    |\n";
        std::cout << "+============================================+\n";
    }
    
    // ================================================================
    // Cleanup
    // ================================================================
    
    cleanup();
    std::cout << "Done!\n";
    
    return 0;
}