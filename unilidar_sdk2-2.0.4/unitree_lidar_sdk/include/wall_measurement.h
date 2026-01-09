/**********************************************************************
 * Wall Measurement Tool for Unitree LiDAR L2
 * Version: 8.2 (GENERIC / UNLOCKED)
 * * CHANGES:
 * - REMOVED fixed expected dimensions.
 * - REMOVED error comparison logic.
 * - Ready for generic wall measurement.
 * * Place in: unitree_lidar_sdk/include/wall_measurement.h
 ***********************************************************************/

#ifndef WALL_MEASUREMENT_H
#define WALL_MEASUREMENT_H

#include "unitree_lidar_sdk.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <random>
#include <fstream>
#include <signal.h>
#include <atomic>

using namespace unitree_lidar_sdk;

// ============================================================================
// SIGNAL HANDLER
// ============================================================================

static std::atomic<bool> g_running(true);

inline void signalHandler(int sig) {
    (void)sig;
    std::cout << "\n*** Stopping... ***\n";
    g_running = false;
}

inline void installSignalHandler() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
}

inline bool isRunning() { return g_running.load(); }

// ============================================================================
// 3D MATH HELPERS
// ============================================================================

struct Point3D {
    double x, y, z;
    
    Point3D(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {}
    
    Point3D operator+(const Point3D& o) const { return Point3D(x + o.x, y + o.y, z + o.z); }
    Point3D operator-(const Point3D& o) const { return Point3D(x - o.x, y - o.y, z - o.z); }
    Point3D operator*(double s) const { return Point3D(x * s, y * s, z * s); }
    Point3D operator/(double s) const { return Point3D(x / s, y / s, z / s); }
    
    double dot(const Point3D& o) const { return x * o.x + y * o.y + z * o.z; }
    
    Point3D cross(const Point3D& o) const {
        return Point3D(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    
    Point3D normalized() const {
        double len = length();
        return (len > 1e-10) ? (*this) / len : Point3D(0, 0, 0);
    }
};

struct Plane {
    Point3D normal;
    double d;
    Plane() : normal(0, 0, 1), d(0) {}
    double signedDistance(const Point3D& p) const { return normal.dot(p) + d; }
    double distance(const Point3D& p) const { return std::abs(signedDistance(p)); }
};

struct WallCorner {
    Point3D pos;
    double u, v;
};

struct WallMeasurement {
    bool valid;
    Plane plane;
    double distance_mm;
    WallCorner corners[4];
    double width_mm;
    double height_mm;
    
    Point3D center;
    Point3D axisU;
    Point3D axisV;
    
    int totalPoints;
    int wallPoints;
    double scanTime_sec;
    double densityFactor;
    
    // Diagnostics
    double planeAngleFromVertical_deg;
    bool positioningWarning;
    
    WallMeasurement() : valid(false), distance_mm(0), width_mm(0), height_mm(0),
                        totalPoints(0), wallPoints(0), scanTime_sec(0), densityFactor(0),
                        planeAngleFromVertical_deg(0), positioningWarning(false) {}
};

struct MeasurementConfig {
    double scanTime_s;
    int minPoints;
    double minDistance_m;  
    double maxDistance_m;
    int ransacIterations;
    double ransacThreshold_m;
    double wallThickness_m;
    double boundaryPercentile;
    bool verbose;
    
    MeasurementConfig() :
        scanTime_s(12.0),           
        minPoints(4000),            
        minDistance_m(0.1),         
        maxDistance_m(8.0),         
        ransacIterations(500),
        ransacThreshold_m(0.025),   // Slightly relaxed (2.5cm)
        wallThickness_m(0.040),     // 4.0cm slice
        boundaryPercentile(0.001),  // 0.1% cut (very minimal)
        verbose(true)
    {}
};

// ============================================================================
// STATISTICAL HELPERS
// ============================================================================

inline double median(std::vector<double>& values) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    size_t n = values.size();
    return (n % 2 == 0) ? (values[n/2-1] + values[n/2]) / 2.0 : values[n/2];
}

inline double mad(std::vector<double>& values) {
    double med = median(values);
    std::vector<double> devs;
    for (double v : values) devs.push_back(std::abs(v - med));
    return median(devs);
}

// ============================================================================
// RANSAC & ALGORITHMS
// ============================================================================

inline Plane fitPlaneRANSAC(const std::vector<Point3D>& points, 
                            int iterations, double threshold, std::vector<int>& inliers) {
    Plane bestPlane;
    int bestCount = 0;
    inliers.clear();
    
    if (points.size() < 3) return bestPlane;
    
    std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);
    
    for (int iter = 0; iter < iterations && isRunning(); iter++) {
        size_t i1 = dist(rng);
        size_t i2 = dist(rng);
        size_t i3 = dist(rng);
        
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;
        
        Point3D v1 = points[i2] - points[i1];
        Point3D v2 = points[i3] - points[i1];
        Point3D normal = v1.cross(v2);
        
        double len = normal.length();
        if (len < 1e-10) continue;
        
        normal = normal / len;
        double d = -normal.dot(points[i1]);
        
        // Count inliers
        int count = 0;
        for (const auto& p : points) {
            if (std::abs(normal.dot(p) + d) < threshold) count++;
        }

        if (count > bestCount) {
            bestCount = count;
            bestPlane.normal = normal;
            bestPlane.d = d;
        }
    }
    
    // Rebuild inliers from best plane
    for (size_t i = 0; i < points.size(); i++) {
        if (std::abs(bestPlane.normal.dot(points[i]) + bestPlane.d) < threshold) {
            inliers.push_back(i);
        }
    }
    
    return bestPlane;
}

class WallMeasurer {
private:
    MeasurementConfig cfg;
    std::vector<Point3D> allPoints;
    std::vector<Point3D> wallPoints;
    
    bool checkPositioning(const std::vector<Point3D>& points) {
        if (points.empty()) return false;
        
        Point3D centroid(0,0,0);
        for (const auto& p : points) centroid = centroid + p;
        centroid = centroid / points.size();
        
        double dist = centroid.length();
        
        if (cfg.verbose) {
            std::cout << "  [Position Check] Average distance: " 
                      << std::fixed << std::setprecision(2) << dist << "m\n";
        }
        
        if (dist < 1.0) { 
            std::cout << "\n  ⚠️  WARNING: LiDAR is very close (" << dist << "m)\n";
            std::cout << "      If the wall is large, corners might be cut off.\n";
            std::cout << "      Proceeding anyway as requested...\n\n";
        }
        
        return true; 
    }
    
public:
    WallMeasurer() {}
    WallMeasurer(const MeasurementConfig& c) : cfg(c) {}
    
    bool collect(UnitreeLidarReader* reader) {
        allPoints.clear();
        if (cfg.verbose) {
            std::cout << "\n[Scanning] UNLOCKED Mode (" << (int)cfg.scanTime_s 
                      << "s). Hold steady...\n";
        }
        
        auto t0 = std::chrono::steady_clock::now();
        int progressMarks = 0;
        
        while (isRunning()) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - t0).count();
            if (elapsed >= cfg.scanTime_s) break;
            
            // Progress indicator
            int newMarks = (int)(elapsed / cfg.scanTime_s * 40);
            while (progressMarks < newMarks && progressMarks < 40) {
                std::cout << "█" << std::flush;
                progressMarks++;
            }
            
            if (reader->runParse() == LIDAR_POINT_DATA_PACKET_TYPE) {
                PointCloudUnitree cloud;
                if (reader->getPointCloud(cloud)) {
                    for (const auto& p : cloud.points) {
                        double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
                        if (dist > 0.05 && dist < cfg.maxDistance_m) {
                            allPoints.push_back(Point3D(p.x, p.y, p.z));
                        }
                    }
                }
            }
        }
        
        std::cout << "\n  Collected " << allPoints.size() << " points\n";
        
        checkPositioning(allPoints);
        
        return (int)allPoints.size() >= cfg.minPoints;
    }
    
    WallMeasurement measure() {
        WallMeasurement result;
        result.totalPoints = allPoints.size();
        
        if (allPoints.size() < 100) {
            std::cout << "  ERROR: Insufficient points\n";
            return result;
        }
        
        // 1. RANSAC PLANE FITTING
        if (cfg.verbose) std::cout << "  > Fitting plane (Tolerance: " 
                                   << (int)(cfg.ransacThreshold_m*1000) << "mm)...\n";
        
        std::vector<int> inliers;
        result.plane = fitPlaneRANSAC(allPoints, cfg.ransacIterations, 
                                      cfg.ransacThreshold_m, inliers);
        
        if (inliers.size() < 500) {
            std::cout << "  ERROR: Plane fit failed (only " << inliers.size() 
                      << " inliers). Try repositioning.\n";
            return result;
        }
        
        // 2. VALIDATE WALL ORIENTATION
        Point3D worldUp(0, 0, 1);
        double angleFromVertical = std::acos(std::abs(result.plane.normal.dot(worldUp))) * 180.0 / M_PI;
        result.planeAngleFromVertical_deg = angleFromVertical;
        
        if (cfg.verbose) {
            std::cout << "  > Plane angle from vertical: " 
                      << std::fixed << std::setprecision(1) << angleFromVertical << "°\n";
        }
        
        // 3. EXTRACT WALL POINTS
        wallPoints.clear();
        Point3D centroid(0,0,0);
        
        for (int idx : inliers) {
            double dist = result.plane.distance(allPoints[idx]);
            if (dist < cfg.wallThickness_m) {
                wallPoints.push_back(allPoints[idx]);
                centroid = centroid + allPoints[idx];
            }
        }
        
        result.wallPoints = wallPoints.size();
        centroid = centroid / result.wallPoints;
        result.center = centroid;
        
        if (cfg.verbose) {
            std::cout << "  > Wall points: " << result.wallPoints << "\n";
        }
        
        // 4. PCA-BASED COORDINATE SYSTEM
        if (cfg.verbose) std::cout << "  > Computing wall coordinate system...\n";
        
        // Use global vertical as reference for consistent orientation
        Point3D globalUp(0, 0, 1);
        if (std::abs(result.plane.normal.dot(globalUp)) > 0.9) {
            globalUp = Point3D(1, 0, 0); // Fallback for horizontal walls
        }
        
        // Right axis (horizontal on wall)
        Point3D tempRight = result.plane.normal.cross(globalUp).normalized();
        // Up axis (vertical on wall)
        Point3D tempUp = tempRight.cross(result.plane.normal).normalized();
        
        // Project points to 2D
        std::vector<std::pair<double, double>> points2D;
        for (const auto& p : wallPoints) {
            Point3D vec = p - centroid;
            points2D.push_back({vec.dot(tempRight), vec.dot(tempUp)});
        }
        
        // PCA rotation for alignment
        double sumUU = 0, sumVV = 0, sumUV = 0;
        for (const auto& pt : points2D) {
            sumUU += pt.first * pt.first;
            sumVV += pt.second * pt.second;
            sumUV += pt.first * pt.second;
        }
        
        double theta = 0.5 * std::atan2(2 * sumUV, sumUU - sumVV);
        result.axisU = (tempRight * std::cos(theta) + tempUp * std::sin(theta)).normalized();
        result.axisV = result.axisU.cross(result.plane.normal).normalized();
        
        // 5. ROBUST BOUNDARY DETECTION
        std::vector<double> uCoords, vCoords;
        for (const auto& p : wallPoints) {
            Point3D vec = p - centroid;
            uCoords.push_back(vec.dot(result.axisU));
            vCoords.push_back(vec.dot(result.axisV));
        }
        
        std::sort(uCoords.begin(), uCoords.end());
        std::sort(vCoords.begin(), vCoords.end());
        
        // Apply minimal percentile cut (0.1% approx)
        int cutIdx = std::max(0, (int)(uCoords.size() * cfg.boundaryPercentile));
        
        double minU = uCoords[cutIdx];
        double maxU = uCoords[uCoords.size() - 1 - cutIdx];
        double minV = vCoords[cutIdx];
        double maxV = vCoords[vCoords.size() - 1 - cutIdx];
        
        // 6. COMPUTE CORNERS
        double us[] = {minU, maxU, maxU, minU};
        double vs[] = {minV, minV, maxV, maxV};
        
        for (int i = 0; i < 4; i++) {
            result.corners[i].u = us[i] * 1000;
            result.corners[i].v = vs[i] * 1000;
            result.corners[i].pos = (centroid + result.axisU * us[i] + 
                                     result.axisV * vs[i]) * 1000;
        }
        
        result.width_mm = (maxU - minU) * 1000;
        result.height_mm = (maxV - minV) * 1000;
        result.distance_mm = std::abs(result.plane.signedDistance(Point3D(0,0,0))) * 1000;
        
        // Ensure width > height (horizontal × vertical)
        if (result.height_mm > result.width_mm) {
            std::swap(result.width_mm, result.height_mm);
            // Also swap corner ordering
            std::swap(result.corners[0], result.corners[1]);
            std::swap(result.corners[2], result.corners[3]);
        }
        
        // Density metrics
        double area_m2 = (result.width_mm/1000.0) * (result.height_mm/1000.0);
        if (area_m2 > 0.01) {
            result.densityFactor = result.wallPoints / area_m2;
        }
        
        result.valid = true;
        return result;
    }
    
    void printResults(const WallMeasurement& m) {
        if (!m.valid) { 
            std::cout << "\n❌ Measurement Failed\n"; 
            return; 
        }
        
        std::cout << "\n╔═══════════════════════════════════════════════════════╗\n";
        std::cout << "║       WALL MEASUREMENT RESULTS v8.2 (GENERIC)         ║\n";
        std::cout << "╠═══════════════════════════════════════════════════════╣\n";
        std::cout << "║                                                       ║\n";
        std::cout << "║  Length (Horizontal): " << std::setw(6) << std::fixed << std::setprecision(0) 
                  << m.width_mm << " mm  (" << std::setprecision(3) 
                  << m.width_mm/1000.0 << " m)  ║\n";
        std::cout << "║  Height (Vertical):   " << std::setw(6) << std::fixed << std::setprecision(0) 
                  << m.height_mm << " mm  (" << std::setprecision(3) 
                  << m.height_mm/1000.0 << " m)  ║\n";
        std::cout << "║  Distance to wall:    " << std::setw(6) << std::fixed << std::setprecision(0) 
                  << m.distance_mm << " mm  (" << std::setprecision(3) 
                  << m.distance_mm/1000.0 << " m)  ║\n";
        std::cout << "║                                                       ║\n";
        std::cout << "╠═══════════════════════════════════════════════════════╣\n";
        
        std::cout << "║  Quality Metrics:                                     ║\n";
        std::cout << "║    Points used: " << std::setw(6) << m.wallPoints << " / " 
                  << std::setw(6) << m.totalPoints << "                         ║\n";
        std::cout << "║    Density: " << std::setw(5) << std::fixed << std::setprecision(0) 
                  << m.densityFactor << " pts/m²                              ║\n";
        
        // FOV warning calculation
        double fovWidth = (m.distance_mm / 1000.0) * 2.2; 
        if (m.width_mm/1000.0 > fovWidth) {
            std::cout << "╠═══════════════════════════════════════════════════════╣\n";
            std::cout << "║  ⚠️  WARNING: FOV CLIPPING LIKELY                       ║\n";
            std::cout << "║      Wall width (" << std::setprecision(2) << m.width_mm/1000.0 
                      << "m) > FOV coverage (~" << fovWidth << "m)       ║\n";
        }
        
        std::cout << "╠═══════════════════════════════════════════════════════╣\n";
        std::cout << "║  Corner Coordinates (mm):                             ║\n";
        std::cout << "║                                                       ║\n";
        const char* labels[] = {"Bottom-Left ", "Bottom-Right", "Top-Right   ", "Top-Left    "};
        for(int i = 0; i < 4; i++) {
            std::cout << "║  " << labels[i] << ": (" 
                      << std::setw(7) << std::fixed << std::setprecision(0) << m.corners[i].pos.x << ", " 
                      << std::setw(7) << m.corners[i].pos.y << ", " 
                      << std::setw(7) << m.corners[i].pos.z << ")  ║\n";
        }
        std::cout << "║                                                       ║\n";
        std::cout << "╚═══════════════════════════════════════════════════════╝\n\n";
    }
    
    void exportVisualization(const WallMeasurement& m, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        file << "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
        file << "<title>Wall Measurement v8.2</title></head><body>";
        file << "<h2>Wall Dimensions</h2>";
        file << "<p><b>Width:</b> " << m.width_mm << " mm (" << m.width_mm/1000.0 << " m)</p>";
        file << "<p><b>Height:</b> " << m.height_mm << " mm (" << m.height_mm/1000.0 << " m)</p>";
        file << "<p><b>Distance:</b> " << m.distance_mm << " mm</p>";
        file << "</body></html>";
        file.close();
    }
};



#endif // WALL_MEASUREMENT_H