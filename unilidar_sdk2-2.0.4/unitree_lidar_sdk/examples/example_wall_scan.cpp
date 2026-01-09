/**********************************************************************
 * Wall Corner Scanner for Unitree LiDAR L2
 * Version: 4.0 (CORNER-BASED)
 * 
 * Detects CORNERS directly, then derives dimensions
 * 
 * Place in: unitree_lidar_sdk/examples/example_wall_scan.cpp
 ***********************************************************************/

#include "unitree_lidar_sdk.h"
#include "wall_feature_detector.h"
#include <unistd.h>

using namespace unitree_lidar_sdk;

UnitreeLidarReader* g_reader = nullptr;

void cleanup() {
    if (g_reader) {
        g_reader->stopLidarRotation();
    }
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    
    installSignalHandler();
    
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  WALL CORNER SCANNER v4.0\n";
    std::cout << "========================================\n";
    std::cout << "\n";
    std::cout << "  *** POINT THE LIDAR AT A WALL ***\n";
    std::cout << "\n";
    std::cout << "  This scanner detects:\n";
    std::cout << "    - CORNERS of features on the wall\n";
    std::cout << "    - Derives dimensions from corners\n";
    std::cout << "\n";
    std::cout << "  Output:\n";
    std::cout << "    - 4 corners for each feature\n";
    std::cout << "    - X, Y, Z coordinates in mm\n";
    std::cout << "    - Width, Height from corners\n";
    std::cout << "\n";
    std::cout << "  Press Ctrl+C to stop.\n";
    std::cout << "\n";
    std::cout << "========================================\n\n";
    
    // Initialize LiDAR
    UnitreeLidarReader* reader = createUnitreeLidarReader();
    g_reader = reader;
    
    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 4000000;
    
    std::cout << "  Connecting to " << port << "...\n";
    
    if (reader->initializeSerial(port, baudrate)) {
        std::cout << "  ERROR: Cannot connect!\n";
        std::cout << "  Check: ls /dev/ttyACM* && sudo chmod 666 /dev/ttyACM0\n";
        return -1;
    }
    
    std::cout << "  Connected!\n";
    
    // Start LiDAR
    if (!isRunning()) { cleanup(); return 0; }
    
    std::cout << "  Starting rotation...\n";
    reader->startLidarRotation();
    sleep(2);
    
    reader->setLidarWorkMode(8);
    sleep(2);
    
    std::cout << "  Stabilizing (5s)...\n";
    sleep(5);
    reader->clearBuffer();
    
    // Wait for data
    if (!isRunning()) { cleanup(); return 0; }
    
    std::cout << "  Checking for data...\n";
    
    bool gotCloud = false;
    for (int attempts = 0; attempts < 40 && !gotCloud && isRunning(); attempts++) {
        for (int i = 0; i < 100 && !gotCloud; i++) {
            if (reader->runParse() == LIDAR_POINT_DATA_PACKET_TYPE) {
                gotCloud = true;
            }
        }
        if (!gotCloud) usleep(500000);
    }
    
    if (!gotCloud) {
        std::cout << "  ERROR: No point cloud!\n";
        cleanup();
        return -1;
    }
    
    std::cout << "  Receiving data!\n";
    
    // Configure detector
    WallDetectorConfig config;
    
    // ============================================================
    // CORNER DETECTION SETTINGS
    // ============================================================
    
    config.scanTime_s = 10.0;
    config.minPoints = 5000;
    
    // Grid resolution - smaller = more precise corners
    config.gridResolution_m = 0.03;     // 3cm cells
    
    // Edge detection
    config.depthEdgeThreshold_m = 0.015;  // 1.5cm depth change = edge
    config.densityEdgeThreshold = 0.25;   // 25% density drop = edge
    
    // Feature filtering
    config.minFeatureSize_mm = 100.0;     // Minimum 10cm feature
    config.maxFeatureSize_mm = 2500.0;    // Maximum 2.5m feature
    config.minConfidence = 0.50;          // 50% confidence threshold
    
    config.verbose = true;
    
    // ============================================================
    
    std::cout << "\n";
    std::cout << "  Configuration:\n";
    std::cout << "    Scan time: " << config.scanTime_s << " seconds\n";
    std::cout << "    Grid resolution: " << config.gridResolution_m * 1000 << " mm\n";
    std::cout << "    Edge threshold: " << config.depthEdgeThreshold_m * 1000 << " mm\n";
    
    // Run scan
    std::cout << "\n";
    std::cout << "  *** POINT AT THE WALL NOW! ***\n\n";
    sleep(2);
    
    WallFeatureDetector detector(config);
    WallScanResult result = detector.scan(reader);
    
    // Print results
    detector.printResults(result);
    
    // Cleanup
    cleanup();
    std::cout << "\nDone!\n";
    
    return 0;
}