/**********************************************************************
 * Ceiling Corner Detector for Unitree LiDAR L2 - IMPROVED VERSION
 * 
 * Purpose: Detect 4 corners of a rectangular ceiling and calculate
 *          precise measurements in millimeters.
 * 
 * FIXES:
 *   - Uses percentile-based ceiling detection (ignores outliers)
 *   - Better diagnostic output
 *   - Handles noisy data better
 * 
 * Place this file in: unitree_lidar_sdk/include/ceiling_corner_detector.h
 ***********************************************************************/

#ifndef CEILING_CORNER_DETECTOR_H
#define CEILING_CORNER_DETECTOR_H

#include "unitree_lidar_sdk.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>

using namespace unitree_lidar_sdk;

// ============================================================================
// GLOBAL SIGNAL HANDLER FOR CTRL+C
// ============================================================================

static std::atomic<bool> g_running(true);
static std::atomic<bool> g_signal_handler_installed(false);

void ceilingDetectorSignalHandler(int signum) {
    std::cout << "\n\n*** Ctrl+C detected! Stopping gracefully... ***\n";
    g_running = false;
}

inline void installSignalHandler() {
    if (!g_signal_handler_installed) {
        signal(SIGINT, ceilingDetectorSignalHandler);
        signal(SIGTERM, ceilingDetectorSignalHandler);
        g_signal_handler_installed = true;
    }
}

inline bool isRunning() {
    return g_running.load();
}

inline void resetRunning() {
    g_running = true;
}

inline void stopRunning() {
    g_running = false;
}

// ============================================================================
// CONFIGURATION PARAMETERS
// ============================================================================

struct CeilingDetectorConfig {
    double collection_time_seconds = 5.0;
    int min_points_required = 1000;
    
    // Ceiling detection - IMPROVED
    double min_ceiling_height_m = 2.0;        // Minimum expected ceiling height
    double max_ceiling_height_m = 4.0;        // Maximum expected ceiling height
    double ceiling_thickness_m = 0.20;        // Thickness tolerance (20cm)
    double percentile_for_ceiling = 0.95;     // Use 95th percentile, not max (ignores outliers)
    
    // RANSAC plane fitting
    int ransac_iterations = 150;
    double ransac_threshold_m = 0.03;         // 3cm threshold
    
    bool verbose = true;
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct Point3D {
    double x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    
    double distanceTo(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    double distanceXY(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

struct Plane {
    double a, b, c, d;
    
    double distanceToPoint(const Point3D& p) const {
        return std::abs(a*p.x + b*p.y + c*p.z + d) / 
               std::sqrt(a*a + b*b + c*c);
    }
};

struct CeilingCorners {
    Point3D corners[4];
    double length_mm;
    double width_mm;
    double diagonal1_mm;
    double diagonal2_mm;
    double area_sqm;
    double ceiling_height_mm;
    bool valid;
    
    CeilingCorners() : length_mm(0), width_mm(0), diagonal1_mm(0), 
                       diagonal2_mm(0), area_sqm(0), ceiling_height_mm(0), valid(false) {}
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

inline double cross2D(const Point2D& O, const Point2D& A, const Point2D& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

inline std::vector<Point2D> convexHull(std::vector<Point2D> points) {
    int n = points.size();
    if (n < 3) return points;
    
    int minIdx = 0;
    for (int i = 1; i < n; i++) {
        if (points[i].y < points[minIdx].y ||
            (points[i].y == points[minIdx].y && points[i].x < points[minIdx].x)) {
            minIdx = i;
        }
    }
    std::swap(points[0], points[minIdx]);
    Point2D pivot = points[0];
    
    std::sort(points.begin() + 1, points.end(), [&pivot](const Point2D& a, const Point2D& b) {
        double v = cross2D(pivot, a, b);
        if (std::abs(v) < 1e-10) {
            double da = (a.x - pivot.x) * (a.x - pivot.x) + (a.y - pivot.y) * (a.y - pivot.y);
            double db = (b.x - pivot.x) * (b.x - pivot.x) + (b.y - pivot.y) * (b.y - pivot.y);
            return da < db;
        }
        return v > 0;
    });
    
    std::vector<Point2D> hull;
    for (int i = 0; i < n; i++) {
        while (hull.size() > 1 && cross2D(hull[hull.size()-2], hull[hull.size()-1], points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }
    
    return hull;
}

inline void minAreaRect(const std::vector<Point2D>& hull, Point2D rect[4]) {
    if (hull.size() < 3) return;
    
    int n = hull.size();
    double minArea = 1e18;
    
    for (int i = 0; i < n; i++) {
        Point2D edge;
        edge.x = hull[(i + 1) % n].x - hull[i].x;
        edge.y = hull[(i + 1) % n].y - hull[i].y;
        double len = std::sqrt(edge.x * edge.x + edge.y * edge.y);
        if (len < 1e-10) continue;
        edge.x /= len;
        edge.y /= len;
        
        Point2D perp;
        perp.x = -edge.y;
        perp.y = edge.x;
        
        double minProj = 1e18, maxProj = -1e18;
        double minPerp = 1e18, maxPerp = -1e18;
        
        for (const auto& p : hull) {
            double proj = p.x * edge.x + p.y * edge.y;
            double perpProj = p.x * perp.x + p.y * perp.y;
            minProj = std::min(minProj, proj);
            maxProj = std::max(maxProj, proj);
            minPerp = std::min(minPerp, perpProj);
            maxPerp = std::max(maxPerp, perpProj);
        }
        
        double width = maxProj - minProj;
        double height = maxPerp - minPerp;
        double area = width * height;
        
        if (area < minArea) {
            minArea = area;
            rect[0].x = minProj * edge.x + minPerp * perp.x;
            rect[0].y = minProj * edge.y + minPerp * perp.y;
            rect[1].x = maxProj * edge.x + minPerp * perp.x;
            rect[1].y = maxProj * edge.y + minPerp * perp.y;
            rect[2].x = maxProj * edge.x + maxPerp * perp.x;
            rect[2].y = maxProj * edge.y + maxPerp * perp.y;
            rect[3].x = minProj * edge.x + maxPerp * perp.x;
            rect[3].y = minProj * edge.y + maxPerp * perp.y;
        }
    }
}

inline Plane fitPlaneRANSAC(const std::vector<Point3D>& points, const CeilingDetectorConfig& config) {
    Plane bestPlane = {0, 0, 1, 0};
    int bestInliers = 0;
    
    if (points.size() < 3) return bestPlane;
    
    srand(42);
    
    for (int iter = 0; iter < config.ransac_iterations && isRunning(); iter++) {
        int i1 = rand() % points.size();
        int i2 = rand() % points.size();
        int i3 = rand() % points.size();
        
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;
        
        const Point3D& p1 = points[i1];
        const Point3D& p2 = points[i2];
        const Point3D& p3 = points[i3];
        
        double v1x = p2.x - p1.x, v1y = p2.y - p1.y, v1z = p2.z - p1.z;
        double v2x = p3.x - p1.x, v2y = p3.y - p1.y, v2z = p3.z - p1.z;
        
        double nx = v1y * v2z - v1z * v2y;
        double ny = v1z * v2x - v1x * v2z;
        double nz = v1x * v2y - v1y * v2x;
        
        double len = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (len < 1e-10) continue;
        
        nx /= len; ny /= len; nz /= len;
        double d = -(nx * p1.x + ny * p1.y + nz * p1.z);
        
        Plane testPlane = {nx, ny, nz, d};
        int inliers = 0;
        for (const auto& p : points) {
            if (testPlane.distanceToPoint(p) < config.ransac_threshold_m) {
                inliers++;
            }
        }
        
        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestPlane = testPlane;
        }
    }
    
    return bestPlane;
}

// Calculate percentile value from a vector
inline double getPercentile(std::vector<double> values, double percentile) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    int index = (int)(percentile * (values.size() - 1));
    return values[index];
}

// ============================================================================
// MAIN CEILING DETECTION CLASS
// ============================================================================

class CeilingCornerDetector {
private:
    CeilingDetectorConfig config;
    std::vector<Point3D> allPoints;
    std::vector<Point3D> ceilingPoints;
    Plane ceilingPlane;
    
public:
    CeilingCornerDetector() {}
    CeilingCornerDetector(const CeilingDetectorConfig& cfg) : config(cfg) {}
    
    void setConfig(const CeilingDetectorConfig& cfg) { config = cfg; }
    
    bool collectData(UnitreeLidarReader* reader) {
        allPoints.clear();
        
        if (config.verbose) {
            std::cout << "\n============================================\n";
            std::cout << "  CEILING CORNER DETECTOR\n";
            std::cout << "============================================\n";
            std::cout << "Collecting point cloud data for " 
                      << config.collection_time_seconds << " seconds...\n";
            std::cout << "Keep the LiDAR stationary!\n";
            std::cout << "Press Ctrl+C anytime to stop.\n\n";
        }
        
        auto startTime = std::chrono::steady_clock::now();
        int cloudCount = 0;
        
        while (isRunning()) {
            auto currentTime = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(currentTime - startTime).count();
            
            if (elapsed >= config.collection_time_seconds) break;
            
            int result = reader->runParse();
            
            if (result == LIDAR_POINT_DATA_PACKET_TYPE) {
                PointCloudUnitree cloud;
                if (reader->getPointCloud(cloud)) {
                    for (const auto& pt : cloud.points) {
                        allPoints.push_back(Point3D(pt.x, pt.y, pt.z));
                    }
                    
                    cloudCount++;
                    
                    if (config.verbose && cloudCount % 10 == 0) {
                        std::cout << "\rCollected " << cloudCount << " scans, " 
                                  << allPoints.size() << " points... " 
                                  << std::fixed << std::setprecision(1) 
                                  << elapsed << "s" << std::flush;
                    }
                }
            }
        }
        
        if (config.verbose) {
            std::cout << "\n\nData collection complete!\n";
            std::cout << "Total scans: " << cloudCount << "\n";
            std::cout << "Total points: " << allPoints.size() << "\n";
        }
        
        return allPoints.size() >= (size_t)config.min_points_required;
    }
    
    bool filterCeilingPoints() {
        ceilingPoints.clear();
        
        if (allPoints.empty()) {
            std::cerr << "Error: No points collected!\n";
            return false;
        }
        
        // Collect all Z values for analysis
        std::vector<double> zValues;
        for (const auto& p : allPoints) {
            zValues.push_back(p.z);
        }
        
        // Calculate statistics
        double minZ = *std::min_element(zValues.begin(), zValues.end());
        double maxZ = *std::max_element(zValues.begin(), zValues.end());
        double sumZ = std::accumulate(zValues.begin(), zValues.end(), 0.0);
        double avgZ = sumZ / zValues.size();
        
        // Use PERCENTILE instead of MAX to find ceiling (ignores outliers!)
        double percentile95 = getPercentile(zValues, 0.95);
        double percentile99 = getPercentile(zValues, 0.99);
        double percentile90 = getPercentile(zValues, 0.90);
        
        if (config.verbose) {
            std::cout << "\n--- Point Cloud Statistics ---\n";
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "Z range: " << minZ << " to " << maxZ << " meters\n";
            std::cout << "Average Z: " << avgZ << " meters\n";
            std::cout << "90th percentile Z: " << percentile90 << " meters\n";
            std::cout << "95th percentile Z: " << percentile95 << " meters\n";
            std::cout << "99th percentile Z: " << percentile99 << " meters\n";
        }
        
        // Determine ceiling height using percentile (not max!)
        // This ignores outlier points like the 6.5m reading you had
        double estimatedCeilingHeight = percentile95;
        
        // Check if estimated ceiling is within expected range
        if (estimatedCeilingHeight < config.min_ceiling_height_m) {
            std::cout << "\nWARNING: Detected ceiling (" << estimatedCeilingHeight 
                      << "m) is below minimum expected (" << config.min_ceiling_height_m << "m)\n";
            std::cout << "Using minimum height as ceiling estimate.\n";
            estimatedCeilingHeight = config.min_ceiling_height_m;
        }
        
        if (estimatedCeilingHeight > config.max_ceiling_height_m) {
            std::cout << "\nWARNING: Detected ceiling (" << estimatedCeilingHeight 
                      << "m) is above maximum expected (" << config.max_ceiling_height_m << "m)\n";
            std::cout << "Using maximum height as ceiling estimate.\n";
            estimatedCeilingHeight = config.max_ceiling_height_m;
        }
        
        // Filter ceiling points: within thickness of estimated ceiling
        double ceilingMinZ = estimatedCeilingHeight - config.ceiling_thickness_m;
        double ceilingMaxZ = estimatedCeilingHeight + config.ceiling_thickness_m;
        
        if (config.verbose) {
            std::cout << "\nUsing ceiling height estimate: " << estimatedCeilingHeight << " m\n";
            std::cout << "Filtering points in range: " << ceilingMinZ << " to " << ceilingMaxZ << " m\n";
        }
        
        for (const auto& p : allPoints) {
            if (p.z >= ceilingMinZ && p.z <= ceilingMaxZ) {
                ceilingPoints.push_back(p);
            }
        }
        
        if (config.verbose) {
            std::cout << "Ceiling points found: " << ceilingPoints.size() << "\n";
        }
        
        // Need at least 100 ceiling points
        if (ceilingPoints.size() < 100) {
            std::cout << "\nERROR: Not enough ceiling points (" << ceilingPoints.size() << ")\n";
            std::cout << "Possible issues:\n";
            std::cout << "  1. LiDAR might not be pointing straight UP at the ceiling\n";
            std::cout << "  2. Ceiling height might be outside the expected range\n";
            std::cout << "  3. Try adjusting min/max ceiling height in config\n";
            std::cout << "\nCurrent config:\n";
            std::cout << "  min_ceiling_height_m = " << config.min_ceiling_height_m << "\n";
            std::cout << "  max_ceiling_height_m = " << config.max_ceiling_height_m << "\n";
            return false;
        }
        
        return true;
    }
    
    bool fitCeilingPlane() {
        if (ceilingPoints.size() < 100) {
            std::cerr << "Error: Not enough ceiling points!\n";
            return false;
        }
        
        ceilingPlane = fitPlaneRANSAC(ceilingPoints, config);
        
        std::vector<Point3D> inlierPoints;
        for (const auto& p : ceilingPoints) {
            if (ceilingPlane.distanceToPoint(p) < config.ransac_threshold_m) {
                inlierPoints.push_back(p);
            }
        }
        ceilingPoints = inlierPoints;
        
        if (config.verbose) {
            std::cout << "\n--- Plane Fitting ---\n";
            std::cout << "Plane normal: (" << std::fixed << std::setprecision(4)
                      << ceilingPlane.a << ", " << ceilingPlane.b << ", "
                      << ceilingPlane.c << ")\n";
            std::cout << "Inlier points: " << ceilingPoints.size() << "\n";
        }
        
        if (ceilingPoints.size() < 50) {
            std::cout << "ERROR: Too few inlier points after plane fitting!\n";
            return false;
        }
        
        return true;
    }
    
    CeilingCorners findCorners() {
        CeilingCorners result;
        
        if (ceilingPoints.size() < 50) {
            std::cerr << "Error: Not enough ceiling points!\n";
            return result;
        }
        
        std::vector<Point2D> points2D;
        double avgZ = 0;
        for (const auto& p : ceilingPoints) {
            points2D.push_back(Point2D(p.x, p.y));
            avgZ += p.z;
        }
        avgZ /= ceilingPoints.size();
        
        if (config.verbose) {
            std::cout << "\n--- Corner Detection ---\n";
            std::cout << "Average ceiling height: " << std::fixed 
                      << std::setprecision(3) << avgZ << " m ("
                      << std::setprecision(1) << avgZ * 1000 << " mm)\n";
        }
        
        std::vector<Point2D> hull = convexHull(points2D);
        
        if (config.verbose) {
            std::cout << "Convex hull vertices: " << hull.size() << "\n";
        }
        
        if (hull.size() < 4) {
            std::cerr << "Error: Not enough hull points!\n";
            return result;
        }
        
        Point2D rect2D[4];
        minAreaRect(hull, rect2D);
        
        for (int i = 0; i < 4; i++) {
            result.corners[i].x = rect2D[i].x;
            result.corners[i].y = rect2D[i].y;
            result.corners[i].z = avgZ;
        }
        
        double cx = 0, cy = 0;
        for (int i = 0; i < 4; i++) { cx += result.corners[i].x; cy += result.corners[i].y; }
        cx /= 4; cy /= 4;
        
        std::vector<std::pair<double, int>> angles;
        for (int i = 0; i < 4; i++) {
            angles.push_back({std::atan2(result.corners[i].y - cy, result.corners[i].x - cx), i});
        }
        std::sort(angles.begin(), angles.end());
        
        Point3D sortedCorners[4];
        for (int i = 0; i < 4; i++) sortedCorners[i] = result.corners[angles[i].second];
        for (int i = 0; i < 4; i++) result.corners[i] = sortedCorners[i];
        
        double d01 = result.corners[0].distanceXY(result.corners[1]) * 1000;
        double d12 = result.corners[1].distanceXY(result.corners[2]) * 1000;
        double d23 = result.corners[2].distanceXY(result.corners[3]) * 1000;
        double d30 = result.corners[3].distanceXY(result.corners[0]) * 1000;
        
        result.length_mm = (d01 + d23) / 2.0;
        result.width_mm = (d12 + d30) / 2.0;
        if (result.width_mm > result.length_mm) std::swap(result.length_mm, result.width_mm);
        
        result.diagonal1_mm = result.corners[0].distanceXY(result.corners[2]) * 1000;
        result.diagonal2_mm = result.corners[1].distanceXY(result.corners[3]) * 1000;
        result.area_sqm = (result.length_mm / 1000.0) * (result.width_mm / 1000.0);
        result.ceiling_height_mm = avgZ * 1000;
        result.valid = true;
        
        return result;
    }
    
    void printResults(const CeilingCorners& corners) {
        if (!corners.valid) {
            std::cout << "\n********************************************\n";
            std::cout << "*          DETECTION FAILED                *\n";
            std::cout << "********************************************\n";
            std::cout << "Could not detect ceiling corners.\n";
            std::cout << "\nTroubleshooting:\n";
            std::cout << "  1. Make sure LiDAR is pointing STRAIGHT UP\n";
            std::cout << "  2. Place LiDAR on a stable, flat surface\n";
            std::cout << "  3. Check that ceiling is within 2-4 meter range\n";
            std::cout << "  4. Make sure nothing blocks the view to ceiling\n";
            return;
        }
        
        std::cout << "\n============================================\n";
        std::cout << "  CEILING MEASUREMENT RESULTS\n";
        std::cout << "============================================\n\n";
        
        std::cout << "--- Corner Coordinates (mm) ---\n";
        std::cout << std::fixed << std::setprecision(1);
        for (int i = 0; i < 4; i++) {
            std::cout << "  Corner " << (i + 1) << ": ("
                      << corners.corners[i].x * 1000 << ", "
                      << corners.corners[i].y * 1000 << ", "
                      << corners.corners[i].z * 1000 << ") mm\n";
        }
        
        std::cout << "\n--- Measurements ---\n";
        std::cout << "  Ceiling Height: " << corners.ceiling_height_mm << " mm ("
                  << std::setprecision(2) << corners.ceiling_height_mm / 1000.0 << " m)\n";
        std::cout << std::setprecision(1);
        std::cout << "  Length:         " << corners.length_mm << " mm ("
                  << std::setprecision(2) << corners.length_mm / 1000.0 << " m)\n";
        std::cout << std::setprecision(1);
        std::cout << "  Width:          " << corners.width_mm << " mm ("
                  << std::setprecision(2) << corners.width_mm / 1000.0 << " m)\n";
        std::cout << "  Diagonal 1:     " << corners.diagonal1_mm << " mm\n";
        std::cout << "  Diagonal 2:     " << corners.diagonal2_mm << " mm\n";
        std::cout << std::setprecision(2);
        std::cout << "  Area:           " << corners.area_sqm << " sq.m ("
                  << corners.area_sqm * 10.7639 << " sq.ft)\n";
        
        double diagonalDiff = std::abs(corners.diagonal1_mm - corners.diagonal2_mm);
        std::cout << "\n--- Quality Check ---\n";
        std::cout << std::setprecision(1);
        std::cout << "  Diagonal difference: " << diagonalDiff << " mm\n";
        if (diagonalDiff < 50) {
            std::cout << "  Rectangle quality: EXCELLENT\n";
        } else if (diagonalDiff < 100) {
            std::cout << "  Rectangle quality: GOOD\n";
        } else {
            std::cout << "  Rectangle quality: FAIR (room may not be perfectly rectangular)\n";
        }
        
        std::cout << "\n============================================\n";
    }
    
    CeilingCorners detect(UnitreeLidarReader* reader) {
        CeilingCorners result;
        
        if (!collectData(reader) || !isRunning()) {
            if (!isRunning()) std::cout << "Stopped by user.\n";
            return result;
        }
        
        if (!filterCeilingPoints()) return result;
        if (!fitCeilingPlane()) return result;
        
        result = findCorners();
        printResults(result);
        
        return result;
    }
};

#endif // CEILING_CORNER_DETECTOR_H