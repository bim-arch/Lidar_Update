/**********************************************************************
 * Wall Measurement Tool for Unitree LiDAR L2
 * Version: 8.2 (GENERIC / UNLOCKED)
 * * Place in: unitree_lidar_sdk/examples/example_wall_measure.cpp
 ***********************************************************************/

#include "unitree_lidar_sdk.h"
#include "wall_measurement.h"
#include <unistd.h>

using namespace unitree_lidar_sdk;

UnitreeLidarReader* g_reader = nullptr;

void cleanup() {
    if (g_reader) {
        std::cout << "\n  Stopping LiDAR...\n";
        g_reader->stopLidarRotation();
    }
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    
    installSignalHandler();
    
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║     WALL MEASUREMENT TOOL v8.2 - GENERIC / UNLOCKED       ║\n";
    std::cout << "╠═══════════════════════════════════════════════════════════╣\n";
    std::cout << "║                                                           ║\n";
    std::cout << "║  SETUP INSTRUCTIONS:                                      ║\n";
    std::cout << "║                                                           ║\n";
    std::cout << "║  1. Point DIRECTLY at wall center (not corners)           ║\n";
    std::cout << "║  2. Ensure entire wall is within FOV (~90° cone)          ║\n";
    std::cout << "║  3. Keep LiDAR LEVEL (not tilted)                         ║\n";
    std::cout << "║  4. Remove obstacles between LiDAR and wall               ║\n";
    std::cout << "║                                                           ║\n";
    std::cout << "║  Press Ctrl+C to cancel at any time.                      ║\n";
    std::cout << "║                                                           ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";
    
    // Initialize LiDAR
    UnitreeLidarReader* reader = createUnitreeLidarReader();
    g_reader = reader;
    
    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 4000000;
    
    std::cout << "  Connecting to " << port << "...\n";
    
    if (reader->initializeSerial(port, baudrate)) {
        std::cout << "  ERROR: Cannot connect to LiDAR!\n";
        std::cout << "\n  Troubleshooting:\n";
        std::cout << "    1. Check cable connection\n";
        std::cout << "    2. Run: ls /dev/ttyACM*\n";
        std::cout << "    3. Run: sudo chmod 666 /dev/ttyACM0\n";
        return -1;
    }
    
    std::cout << "  ✓ Connected!\n";
    
    // Start LiDAR motor
    if (!isRunning()) { cleanup(); return 0; }
    
    std::cout << "  Starting LiDAR motor...\n";
    reader->startLidarRotation();
    sleep(2);
    
    // Set work mode
    reader->setLidarWorkMode(8);
    sleep(2);
    
    // Stabilization
    std::cout << "  Stabilizing (5 seconds)...\n";
    sleep(5);
    reader->clearBuffer();
    
    if (!isRunning()) { cleanup(); return 0; }
    
    // Check for data
    std::cout << "  Checking for point cloud data...\n";
    
    bool gotData = false;
    for (int attempt = 0; attempt < 50 && !gotData && isRunning(); attempt++) {
        for (int i = 0; i < 100 && !gotData; i++) {
            if (reader->runParse() == LIDAR_POINT_DATA_PACKET_TYPE) {
                gotData = true;
            }
        }
        if (!gotData) usleep(200000);
    }
    
    if (!gotData) {
        std::cout << "  ERROR: No point cloud data received!\n";
        cleanup();
        return -1;
    }
    
    std::cout << "  ✓ Point cloud data OK!\n";
    
    // Configure measurement
    MeasurementConfig config;
    config.scanTime_s = 12.0;           // 12 seconds for thorough coverage
    config.minPoints = 8000;            // Higher quality threshold
    config.maxDistance_m = 6.0;         // Reasonable maximum
    config.ransacIterations = 500;      // More iterations for better fit
    config.ransacThreshold_m = 0.020;   // 2cm tolerance (stricter)
    config.wallThickness_m = 0.035;     // 3.5cm slice depth
    config.boundaryPercentile = 0.0025; // Minimal 0.25% cut
    config.verbose = true;
    
    std::cout << "\n";
    std::cout << "  ╭─────────────────────────────────────────────────╮\n";
    std::cout << "  │                                                 │\n";
    std::cout << "  │  ⚠️  VERIFY POSITIONING BEFORE SCAN:             │\n";
    std::cout << "  │                                                 │\n";
    std::cout << "  │  • Aim: Wall center (not edges)                │\n";
    std::cout << "  │  • Level: LiDAR horizontal                      │\n";
    std::cout << "  │                                                 │\n";
    std::cout << "  │  Scanning starts in 5 seconds...                │\n";
    std::cout << "  │                                                 │\n";
    std::cout << "  ╰─────────────────────────────────────────────────╯\n";
    std::cout << "\n";
    
    sleep(5);
    
    if (!isRunning()) { cleanup(); return 0; }
    
    // Create measurer and run
    WallMeasurer measurer(config);
    
    // Collect points
    if (!measurer.collect(reader)) {
        std::cout << "\n  ⛔ Collection failed - Check positioning and retry\n";
        cleanup();
        return -1;
    }
    
    // Stop LiDAR motor
    std::cout << "\n  Stopping LiDAR motor...\n";
    reader->stopLidarRotation();
    
    if (!isRunning()) { cleanup(); return 0; }
    
    // Process and measure
    std::cout << "\n  Processing point cloud...\n";
    WallMeasurement result = measurer.measure();
    
    // Print results
    measurer.printResults(result);
    
    // Export visualization
    if (result.valid) {
        std::string vizFile = "/tmp/wall_measurement_v8.html";
        measurer.exportVisualization(result, vizFile);
        std::cout << "  Visualization saved to: " << vizFile << "\n";
    }
    
    std::cout << "\nDone!\n\n";
    
    return 0;
}