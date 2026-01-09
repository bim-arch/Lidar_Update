/**********************************************************************
 * Wall Corner Detector for Unitree LiDAR L2
 * Version: 4.0 (CORNER-BASED DETECTION)
 * 
 * APPROACH: Detect CORNERS directly where coordinates change suddenly
 *           Then derive all dimensions from corner positions
 * 
 * HOW IT WORKS:
 *   1. Create 2D depth map on wall surface
 *   2. Find EDGES where depth/density changes suddenly
 *   3. Extract CORNERS where edges meet at angles
 *   4. Group corners into rectangular features
 *   5. Output precise corner coordinates
 * 
 * OUTPUT:
 *   - Wall corners (4 points)
 *   - Feature corners (4 points each)
 *   - Derived dimensions from corners
 * 
 * Place in: unitree_lidar_sdk/include/wall_feature_detector.h
 ***********************************************************************/

#ifndef WALL_FEATURE_DETECTOR_H
#define WALL_FEATURE_DETECTOR_H

#include "unitree_lidar_sdk.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <map>
#include <set>
#include <queue>
#include <tuple>
#include <random>
#include <cstring>
#include <signal.h>
#include <atomic>
#include <climits>

using namespace unitree_lidar_sdk;

// ============================================================================
// SIGNAL HANDLER
// ============================================================================

static std::atomic<bool> g_running(true);
static std::atomic<bool> g_sig_installed(false);

inline void wallSignalHandler(int sig) {
    (void)sig;
    std::cout << "\n*** Stopping... ***\n";
    g_running = false;
}

inline void installSignalHandler() {
    if (!g_sig_installed) {
        signal(SIGINT, wallSignalHandler);
        signal(SIGTERM, wallSignalHandler);
        g_sig_installed = true;
    }
}

inline bool isRunning() { return g_running.load(); }

// ============================================================================
// FEATURE TYPE (for optional classification)
// ============================================================================

enum class FeatureType {
    UNKNOWN = 0,
    HOLE,           // Empty space (door, window, opening)
    PROTRUSION      // Something sticking out (beam, column, switch)
};

inline const char* featureTypeName(FeatureType t) {
    switch(t) {
        case FeatureType::HOLE: return "HOLE";
        case FeatureType::PROTRUSION: return "PROTRUSION";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// 3D POINT
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
    
    double dist(const Point3D& o) const { return (*this - o).length(); }
};

// ============================================================================
// PLANE
// ============================================================================

struct Plane {
    Point3D normal;
    double d;
    
    Plane() : normal(0, 1, 0), d(0) {}
    Plane(const Point3D& n, double _d) : normal(n.normalized()), d(_d) {}
    
    double signedDistance(const Point3D& p) const { return normal.dot(p) + d; }
    double distance(const Point3D& p) const { return std::abs(signedDistance(p)); }
    Point3D project(const Point3D& p) const { return p - normal * signedDistance(p); }
};

// ============================================================================
// CORNER POINT (with confidence)
// ============================================================================

struct Corner {
    Point3D pos_mm;         // 3D position in mm
    double wallX_mm;        // X position on wall (horizontal)
    double wallZ_mm;        // Z position on wall (vertical/height)
    double confidence;      // 0-1
    bool valid;
    
    Corner() : wallX_mm(0), wallZ_mm(0), confidence(0), valid(false) {}
};

// ============================================================================
// DETECTED FEATURE (4 corners)
// ============================================================================

struct DetectedFeature {
    FeatureType type;
    
    // 4 corners: BL(0), BR(1), TR(2), TL(3)
    Corner corners[4];
    
    // Derived from corners (all in mm)
    double width_mm;        // BR.x - BL.x
    double height_mm;       // TL.z - BL.z
    double depth_mm;        // For protrusions
    
    double fromFloor_mm;    // BL.z - wall.minZ
    double fromCeiling_mm;  // wall.maxZ - TL.z
    double fromLeft_mm;     // BL.x - wall.minX
    double fromRight_mm;    // wall.maxX - BR.x
    
    double confidence;
    int pointCount;
    
    DetectedFeature() : type(FeatureType::UNKNOWN), 
                        width_mm(0), height_mm(0), depth_mm(0),
                        fromFloor_mm(0), fromCeiling_mm(0),
                        fromLeft_mm(0), fromRight_mm(0),
                        confidence(0), pointCount(0) {}
};

// ============================================================================
// WALL INFO
// ============================================================================

struct WallInfo {
    Plane plane;
    
    // Wall boundaries in wall coordinates (meters)
    double minX, maxX;  // Horizontal
    double minZ, maxZ;  // Vertical (floor to ceiling)
    double avgDepth;    // Distance from LiDAR
    
    // Wall dimensions in mm
    double width_mm;
    double height_mm;
    
    // 4 corners of wall in 3D (mm)
    Corner corners[4];  // BL, BR, TR, TL
    
    // Coordinate axes on wall
    Point3D rightAxis;  // Horizontal direction
    Point3D upAxis;     // Vertical direction
    
    bool valid;
    
    WallInfo() : minX(0), maxX(0), minZ(0), maxZ(0), avgDepth(0),
                 width_mm(0), height_mm(0), valid(false) {}
};

// ============================================================================
// SCAN RESULT
// ============================================================================

struct WallScanResult {
    WallInfo wall;
    std::vector<DetectedFeature> features;
    
    int totalPoints;
    int wallPoints;
    double scanSeconds;
    
    bool valid;
    
    WallScanResult() : totalPoints(0), wallPoints(0), scanSeconds(0), valid(false) {}
};

// ============================================================================
// CONFIGURATION
// ============================================================================

struct WallDetectorConfig {
    // Scan
    double scanTime_s;
    int minPoints;
    double maxDistance_m;
    
    // RANSAC
    int ransacIterations;
    double ransacThreshold_m;
    int ransacMinInliers;
    
    // Wall
    double wallThickness_m;
    
    // Grid for edge detection
    double gridResolution_m;    // Cell size (smaller = more precise)
    
    // Edge detection thresholds
    double depthEdgeThreshold_m;    // Depth change to consider an edge
    double densityEdgeThreshold;    // Density ratio to consider an edge (0-1)
    int minEdgePoints;              // Minimum points to form an edge
    
    // Corner detection
    double cornerAngleThreshold;    // Angle threshold for corner detection (degrees)
    int minCornerPoints;
    
    // Feature filtering
    double minFeatureSize_mm;       // Minimum feature dimension
    double maxFeatureSize_mm;       // Maximum feature dimension
    double minConfidence;           // Minimum confidence to report
    
    bool verbose;
    
    WallDetectorConfig() :
        scanTime_s(10.0),
        minPoints(5000),
        maxDistance_m(6.0),
        
        ransacIterations(200),
        ransacThreshold_m(0.02),
        ransacMinInliers(1000),
        
        wallThickness_m(0.03),
        
        gridResolution_m(0.03),     // 3cm cells for precision
        
        depthEdgeThreshold_m(0.02), // 2cm depth change = edge
        densityEdgeThreshold(0.3),  // 30% density drop = edge
        minEdgePoints(3),
        
        cornerAngleThreshold(45.0),
        minCornerPoints(2),
        
        minFeatureSize_mm(100.0),   // Min 10cm
        maxFeatureSize_mm(3000.0),  // Max 3m
        minConfidence(0.5),
        
        verbose(true)
    {}
};

// ============================================================================
// DEPTH MAP CELL
// ============================================================================

struct DepthCell {
    int pointCount;
    double sumDepth;
    double avgDepth;
    bool hasPoints;
    bool isEdge;
    int edgeType;  // 0=none, 1=depth, 2=density, 3=both
    
    DepthCell() : pointCount(0), sumDepth(0), avgDepth(0), 
                  hasPoints(false), isEdge(false), edgeType(0) {}
    
    void addPoint(double depth) {
        pointCount++;
        sumDepth += depth;
        hasPoints = true;
    }
    
    void finalize() {
        if (pointCount > 0) {
            avgDepth = sumDepth / pointCount;
        }
    }
};

// ============================================================================
// DEPTH MAP
// ============================================================================

class DepthMap {
public:
    int rows, cols;
    double cellSize;
    double originX, originZ;
    std::vector<std::vector<DepthCell>> cells;
    double avgPointsPerCell;
    
    DepthMap(double minX, double maxX, double minZ, double maxZ, double resolution) {
        cellSize = resolution;
        originX = minX;
        originZ = minZ;
        cols = std::max(1, (int)std::ceil((maxX - minX) / cellSize));
        rows = std::max(1, (int)std::ceil((maxZ - minZ) / cellSize));
        cells.resize(rows, std::vector<DepthCell>(cols));
        avgPointsPerCell = 0;
    }
    
    bool getCell(double x, double z, int& row, int& col) const {
        col = (int)((x - originX) / cellSize);
        row = (int)((z - originZ) / cellSize);
        return (row >= 0 && row < rows && col >= 0 && col < cols);
    }
    
    void addPoint(double x, double z, double depth) {
        int row, col;
        if (getCell(x, z, row, col)) {
            cells[row][col].addPoint(depth);
        }
    }
    
    void finalize() {
        int totalCells = 0;
        int totalPoints = 0;
        
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                cells[r][c].finalize();
                if (cells[r][c].hasPoints) {
                    totalCells++;
                    totalPoints += cells[r][c].pointCount;
                }
            }
        }
        
        avgPointsPerCell = (totalCells > 0) ? (double)totalPoints / totalCells : 0;
    }
    
    // Get world coordinates for cell center
    void getCellCenter(int row, int col, double& x, double& z) const {
        x = originX + (col + 0.5) * cellSize;
        z = originZ + (row + 0.5) * cellSize;
    }
    
    // Get cell corner coordinates
    void getCellCorners(int row, int col, double& x1, double& z1, double& x2, double& z2) const {
        x1 = originX + col * cellSize;
        z1 = originZ + row * cellSize;
        x2 = x1 + cellSize;
        z2 = z1 + cellSize;
    }
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

inline double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t idx = std::min((size_t)(p * v.size()), v.size() - 1);
    return v[idx];
}

inline double median(std::vector<double> v) {
    return percentile(v, 0.5);
}

// ============================================================================
// RANSAC PLANE FITTING
// ============================================================================

inline Plane fitPlaneRANSAC(const std::vector<Point3D>& points, 
                            int iterations, double threshold, 
                            std::vector<int>& inlierIndices) {
    Plane bestPlane;
    int bestInliers = 0;
    
    if (points.size() < 3) return bestPlane;
    
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);
    
    for (int iter = 0; iter < iterations && isRunning(); iter++) {
        size_t i1 = dist(rng);
        size_t i2 = dist(rng);
        size_t i3 = dist(rng);
        
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;
        
        Point3D v1 = points[i2] - points[i1];
        Point3D v2 = points[i3] - points[i1];
        Point3D normal = v1.cross(v2);
        
        if (normal.length() < 1e-10) continue;
        normal = normal.normalized();
        
        double d = -normal.dot(points[i1]);
        Plane testPlane(normal, d);
        
        std::vector<int> currentInliers;
        for (size_t i = 0; i < points.size(); i++) {
            if (testPlane.distance(points[i]) < threshold) {
                currentInliers.push_back(i);
            }
        }
        
        if ((int)currentInliers.size() > bestInliers) {
            bestInliers = currentInliers.size();
            bestPlane = testPlane;
            inlierIndices = currentInliers;
        }
    }
    
    return bestPlane;
}

// ============================================================================
// MAIN DETECTOR CLASS
// ============================================================================

class WallFeatureDetector {
private:
    WallDetectorConfig cfg;
    std::vector<Point3D> allPoints;
    std::vector<Point3D> wallPoints;
    WallInfo wall;
    DepthMap* depthMap;
    
public:
    WallFeatureDetector() : depthMap(nullptr) {}
    WallFeatureDetector(const WallDetectorConfig& c) : cfg(c), depthMap(nullptr) {}
    ~WallFeatureDetector() { if (depthMap) delete depthMap; }
    
    void setConfig(const WallDetectorConfig& c) { cfg = c; }
    
    // ========================================================================
    // DATA COLLECTION
    // ========================================================================
    
    bool collect(UnitreeLidarReader* reader) {
        allPoints.clear();
        
        if (cfg.verbose) {
            std::cout << "\n";
            std::cout << "========================================\n";
            std::cout << "  WALL CORNER DETECTOR v4.0\n";
            std::cout << "========================================\n";
            std::cout << "  Point LiDAR at WALL and hold steady!\n";
            std::cout << "  Scanning for " << cfg.scanTime_s << " seconds...\n\n";
        }
        
        auto t0 = std::chrono::steady_clock::now();
        int scans = 0;
        
        while (isRunning()) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - t0).count();
            if (elapsed >= cfg.scanTime_s) break;
            
            int r = reader->runParse();
            if (r == LIDAR_POINT_DATA_PACKET_TYPE) {
                PointCloudUnitree cloud;
                if (reader->getPointCloud(cloud)) {
                    for (size_t i = 0; i < cloud.points.size(); i++) {
                        Point3D pt(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
                        double dist = pt.length();
                        if (dist > 0.1 && dist < cfg.maxDistance_m) {
                            allPoints.push_back(pt);
                        }
                    }
                    scans++;
                    
                    if (cfg.verbose && scans % 10 == 0) {
                        int pct = (int)(elapsed / cfg.scanTime_s * 100);
                        std::cout << "\r  Progress: " << std::setw(3) << pct << "% | "
                                  << "Points: " << std::setw(7) << allPoints.size() << std::flush;
                    }
                }
            }
        }
        
        if (cfg.verbose) {
            std::cout << "\n  Collection complete: " << allPoints.size() << " points\n";
        }
        
        return allPoints.size() >= (size_t)cfg.minPoints;
    }
    
    // ========================================================================
    // WALL PLANE DETECTION
    // ========================================================================
    
    bool detectWallPlane() {
        if (allPoints.size() < 100) return false;
        
        if (cfg.verbose) std::cout << "\n  [1] Detecting wall plane...\n";
        
        std::vector<int> inliers;
        wall.plane = fitPlaneRANSAC(allPoints, cfg.ransacIterations, 
                                     cfg.ransacThreshold_m, inliers);
        
        if ((int)inliers.size() < cfg.ransacMinInliers) {
            std::cout << "  ERROR: Not enough wall points!\n";
            return false;
        }
        
        wallPoints.clear();
        for (size_t i = 0; i < inliers.size(); i++) {
            int idx = inliers[i];
            if (wall.plane.distance(allPoints[idx]) < cfg.wallThickness_m) {
                wallPoints.push_back(allPoints[idx]);
            }
        }
        
        if (cfg.verbose) std::cout << "      Wall points: " << wallPoints.size() << "\n";
        
        return wallPoints.size() > 100;
    }
    
    // ========================================================================
    // COMPUTE WALL BOUNDARIES AND COORDINATE SYSTEM
    // ========================================================================
    
    bool computeWallBoundaries() {
        if (wallPoints.empty()) return false;
        
        if (cfg.verbose) std::cout << "\n  [2] Computing wall boundaries...\n";
        
        // Set up wall coordinate system
        Point3D worldUp(0, 0, 1);
        wall.rightAxis = wall.plane.normal.cross(worldUp).normalized();
        wall.upAxis = wall.rightAxis.cross(wall.plane.normal).normalized();
        
        if (wall.upAxis.z < 0) {
            wall.upAxis = wall.upAxis * -1;
            wall.rightAxis = wall.rightAxis * -1;
        }
        
        // Project points onto wall and find boundaries
        std::vector<double> xCoords, zCoords, depths;
        
        for (size_t i = 0; i < wallPoints.size(); i++) {
            Point3D proj = wall.plane.project(wallPoints[i]);
            xCoords.push_back(wall.rightAxis.dot(proj));
            zCoords.push_back(wall.upAxis.dot(proj));
            depths.push_back(wall.plane.signedDistance(wallPoints[i]));
        }
        
        // Use percentiles for robust boundary estimation
        wall.minX = percentile(xCoords, 0.01);
        wall.maxX = percentile(xCoords, 0.99);
        wall.minZ = percentile(zCoords, 0.01);
        wall.maxZ = percentile(zCoords, 0.99);
        wall.avgDepth = std::abs(median(depths));
        
        wall.width_mm = (wall.maxX - wall.minX) * 1000;
        wall.height_mm = (wall.maxZ - wall.minZ) * 1000;
        
        // Compute wall corners in 3D
        Point3D center = wall.plane.project(Point3D(0, 0, 0));
        
        Point3D bl = center + wall.rightAxis * wall.minX + wall.upAxis * wall.minZ;
        Point3D br = center + wall.rightAxis * wall.maxX + wall.upAxis * wall.minZ;
        Point3D tr = center + wall.rightAxis * wall.maxX + wall.upAxis * wall.maxZ;
        Point3D tl = center + wall.rightAxis * wall.minX + wall.upAxis * wall.maxZ;
        
        wall.corners[0].pos_mm = bl * 1000;
        wall.corners[0].wallX_mm = wall.minX * 1000;
        wall.corners[0].wallZ_mm = wall.minZ * 1000;
        wall.corners[0].confidence = 1.0;
        wall.corners[0].valid = true;
        
        wall.corners[1].pos_mm = br * 1000;
        wall.corners[1].wallX_mm = wall.maxX * 1000;
        wall.corners[1].wallZ_mm = wall.minZ * 1000;
        wall.corners[1].confidence = 1.0;
        wall.corners[1].valid = true;
        
        wall.corners[2].pos_mm = tr * 1000;
        wall.corners[2].wallX_mm = wall.maxX * 1000;
        wall.corners[2].wallZ_mm = wall.maxZ * 1000;
        wall.corners[2].confidence = 1.0;
        wall.corners[2].valid = true;
        
        wall.corners[3].pos_mm = tl * 1000;
        wall.corners[3].wallX_mm = wall.minX * 1000;
        wall.corners[3].wallZ_mm = wall.maxZ * 1000;
        wall.corners[3].confidence = 1.0;
        wall.corners[3].valid = true;
        
        wall.valid = (wall.width_mm > 500 && wall.height_mm > 500);
        
        if (cfg.verbose) {
            std::cout << std::fixed << std::setprecision(0);
            std::cout << "      Wall: " << wall.width_mm << " x " << wall.height_mm << " mm\n";
        }
        
        return wall.valid;
    }
    
    // ========================================================================
    // BUILD DEPTH MAP
    // ========================================================================
    
    bool buildDepthMap() {
        if (!wall.valid) return false;
        
        if (cfg.verbose) std::cout << "\n  [3] Building depth map...\n";
        
        if (depthMap) delete depthMap;
        depthMap = new DepthMap(wall.minX, wall.maxX, wall.minZ, wall.maxZ, cfg.gridResolution_m);
        
        // Add all points (wall + protruding)
        for (size_t i = 0; i < allPoints.size() && isRunning(); i++) {
            Point3D proj = wall.plane.project(allPoints[i]);
            double x = wall.rightAxis.dot(proj);
            double z = wall.upAxis.dot(proj);
            double depth = wall.plane.signedDistance(allPoints[i]);
            
            // Only add points within wall bounds
            if (x >= wall.minX && x <= wall.maxX && z >= wall.minZ && z <= wall.maxZ) {
                depthMap->addPoint(x, z, depth);
            }
        }
        
        depthMap->finalize();
        
        if (cfg.verbose) {
            std::cout << "      Grid: " << depthMap->cols << " x " << depthMap->rows << " cells\n";
            std::cout << "      Avg points/cell: " << std::setprecision(1) << depthMap->avgPointsPerCell << "\n";
        }
        
        return true;
    }
    
    // ========================================================================
    // DETECT EDGES (where depth or density changes)
    // ========================================================================
    
    void detectEdges() {
        if (!depthMap) return;
        
        if (cfg.verbose) std::cout << "\n  [4] Detecting edges...\n";
        
        int edgeCount = 0;
        
        for (int r = 1; r < depthMap->rows - 1 && isRunning(); r++) {
            for (int c = 1; c < depthMap->cols - 1; c++) {
                DepthCell& cell = depthMap->cells[r][c];
                
                bool depthEdge = false;
                bool densityEdge = false;
                
                // Check neighbors for depth/density changes
                int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
                
                for (int n = 0; n < 4; n++) {
                    int nr = r + neighbors[n][0];
                    int nc = c + neighbors[n][1];
                    DepthCell& neighbor = depthMap->cells[nr][nc];
                    
                    // Depth edge: significant depth difference
                    if (cell.hasPoints && neighbor.hasPoints) {
                        double depthDiff = std::abs(cell.avgDepth - neighbor.avgDepth);
                        if (depthDiff > cfg.depthEdgeThreshold_m) {
                            depthEdge = true;
                        }
                    }
                    
                    // Density edge: one has points, other doesn't (or much fewer)
                    if (cell.hasPoints && !neighbor.hasPoints) {
                        densityEdge = true;
                    }
                    if (!cell.hasPoints && neighbor.hasPoints) {
                        densityEdge = true;
                    }
                    if (cell.hasPoints && neighbor.hasPoints) {
                        double ratio = (double)std::min(cell.pointCount, neighbor.pointCount) / 
                                      std::max(cell.pointCount, neighbor.pointCount);
                        if (ratio < cfg.densityEdgeThreshold) {
                            densityEdge = true;
                        }
                    }
                }
                
                if (depthEdge || densityEdge) {
                    cell.isEdge = true;
                    cell.edgeType = (depthEdge ? 1 : 0) + (densityEdge ? 2 : 0);
                    edgeCount++;
                }
            }
        }
        
        if (cfg.verbose) {
            std::cout << "      Edge cells: " << edgeCount << "\n";
        }
    }
    
    // ========================================================================
    // FIND RECTANGULAR REGIONS (potential features)
    // ========================================================================
    
    std::vector<DetectedFeature> findFeatures() {
        std::vector<DetectedFeature> features;
        if (!depthMap) return features;
        
        if (cfg.verbose) std::cout << "\n  [5] Finding features...\n";
        
        // Find connected regions of empty cells (holes) and protruding cells
        std::vector<std::vector<bool>> visited(depthMap->rows, std::vector<bool>(depthMap->cols, false));
        
        // First pass: find HOLES (empty regions)
        for (int r = 0; r < depthMap->rows && isRunning(); r++) {
            for (int c = 0; c < depthMap->cols && isRunning(); c++) {
                if (visited[r][c]) continue;
                
                DepthCell& cell = depthMap->cells[r][c];
                
                // Check for hole (no points or very few)
                if (!cell.hasPoints || cell.pointCount < depthMap->avgPointsPerCell * 0.2) {
                    // Flood fill to find connected empty region
                    std::vector<std::pair<int,int>> region;
                    std::queue<std::pair<int,int>> queue;
                    queue.push({r, c});
                    visited[r][c] = true;
                    
                    while (!queue.empty() && isRunning()) {
                        auto [cr, cc] = queue.front();
                        queue.pop();
                        region.push_back({cr, cc});
                        
                        // 4-connected neighbors
                        int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
                        for (int n = 0; n < 4; n++) {
                            int nr = cr + neighbors[n][0];
                            int nc = cc + neighbors[n][1];
                            
                            if (nr >= 0 && nr < depthMap->rows && nc >= 0 && nc < depthMap->cols &&
                                !visited[nr][nc]) {
                                DepthCell& ncell = depthMap->cells[nr][nc];
                                if (!ncell.hasPoints || ncell.pointCount < depthMap->avgPointsPerCell * 0.2) {
                                    visited[nr][nc] = true;
                                    queue.push({nr, nc});
                                }
                            }
                        }
                    }
                    
                    // Convert region to feature if large enough
                    if (region.size() >= 4) {
                        DetectedFeature f = regionToFeature(region, FeatureType::HOLE);
                        if (f.confidence >= cfg.minConfidence && isValidFeatureSize(f)) {
                            features.push_back(f);
                        }
                    }
                }
            }
        }
        
        // Reset visited for protrusion detection
        for (int r = 0; r < depthMap->rows; r++) {
            for (int c = 0; c < depthMap->cols; c++) {
                visited[r][c] = false;
            }
        }
        
        // Second pass: find PROTRUSIONS (cells with points in front of wall)
        double wallDepthThreshold = cfg.depthEdgeThreshold_m;
        
        for (int r = 0; r < depthMap->rows && isRunning(); r++) {
            for (int c = 0; c < depthMap->cols && isRunning(); c++) {
                if (visited[r][c]) continue;
                
                DepthCell& cell = depthMap->cells[r][c];
                
                // Check for protrusion (points significantly in front of wall)
                if (cell.hasPoints && cell.avgDepth > wallDepthThreshold) {
                    std::vector<std::pair<int,int>> region;
                    std::queue<std::pair<int,int>> queue;
                    queue.push({r, c});
                    visited[r][c] = true;
                    
                    while (!queue.empty() && isRunning()) {
                        auto [cr, cc] = queue.front();
                        queue.pop();
                        region.push_back({cr, cc});
                        
                        int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
                        for (int n = 0; n < 4; n++) {
                            int nr = cr + neighbors[n][0];
                            int nc = cc + neighbors[n][1];
                            
                            if (nr >= 0 && nr < depthMap->rows && nc >= 0 && nc < depthMap->cols &&
                                !visited[nr][nc]) {
                                DepthCell& ncell = depthMap->cells[nr][nc];
                                if (ncell.hasPoints && ncell.avgDepth > wallDepthThreshold) {
                                    visited[nr][nc] = true;
                                    queue.push({nr, nc});
                                }
                            }
                        }
                    }
                    
                    if (region.size() >= 4) {
                        DetectedFeature f = regionToFeature(region, FeatureType::PROTRUSION);
                        if (f.confidence >= cfg.minConfidence && isValidFeatureSize(f)) {
                            features.push_back(f);
                        }
                    }
                }
            }
        }
        
        if (cfg.verbose) {
            std::cout << "      Features found: " << features.size() << "\n";
        }
        
        return features;
    }
    
    // ========================================================================
    // CONVERT REGION TO FEATURE WITH PRECISE CORNERS
    // ========================================================================
    
    DetectedFeature regionToFeature(const std::vector<std::pair<int,int>>& region, FeatureType type) {
        DetectedFeature f;
        f.type = type;
        
        // Find bounding box in grid coordinates
        int minR = 999999;
        int maxR = -1;
        int minC = 999999;
        int maxC = -1;
        int totalPoints = 0;
        double maxDepth = 0;
        
        for (const auto& [r, c] : region) {
            minR = std::min(minR, r);
            maxR = std::max(maxR, r);
            minC = std::min(minC, c);
            maxC = std::max(maxC, c);
            totalPoints += depthMap->cells[r][c].pointCount;
            if (depthMap->cells[r][c].hasPoints) {
                maxDepth = std::max(maxDepth, depthMap->cells[r][c].avgDepth);
            }
        }
        
        f.pointCount = totalPoints;
        
        // Convert to wall coordinates (in meters)
        double x1, z1, x2, z2;
        depthMap->getCellCorners(minR, minC, x1, z1, x2, z2);  // BL corner
        double blX = x1, blZ = z1;
        
        depthMap->getCellCorners(maxR, maxC, x1, z1, x2, z2);  // TR corner
        double trX = x2, trZ = z2;
        
        // Compute 3D positions for all 4 corners
        Point3D center = wall.plane.project(Point3D(0, 0, 0));
        
        // BL corner
        f.corners[0].wallX_mm = blX * 1000;
        f.corners[0].wallZ_mm = blZ * 1000;
        f.corners[0].pos_mm = (center + wall.rightAxis * blX + wall.upAxis * blZ) * 1000;
        f.corners[0].confidence = 0.9;
        f.corners[0].valid = true;
        
        // BR corner
        f.corners[1].wallX_mm = trX * 1000;
        f.corners[1].wallZ_mm = blZ * 1000;
        f.corners[1].pos_mm = (center + wall.rightAxis * trX + wall.upAxis * blZ) * 1000;
        f.corners[1].confidence = 0.9;
        f.corners[1].valid = true;
        
        // TR corner
        f.corners[2].wallX_mm = trX * 1000;
        f.corners[2].wallZ_mm = trZ * 1000;
        f.corners[2].pos_mm = (center + wall.rightAxis * trX + wall.upAxis * trZ) * 1000;
        f.corners[2].confidence = 0.9;
        f.corners[2].valid = true;
        
        // TL corner
        f.corners[3].wallX_mm = blX * 1000;
        f.corners[3].wallZ_mm = trZ * 1000;
        f.corners[3].pos_mm = (center + wall.rightAxis * blX + wall.upAxis * trZ) * 1000;
        f.corners[3].confidence = 0.9;
        f.corners[3].valid = true;
        
        // Derive dimensions from corners
        f.width_mm = f.corners[1].wallX_mm - f.corners[0].wallX_mm;
        f.height_mm = f.corners[3].wallZ_mm - f.corners[0].wallZ_mm;
        f.depth_mm = maxDepth * 1000;
        
        // Position relative to wall
        f.fromFloor_mm = f.corners[0].wallZ_mm - wall.minZ * 1000;
        f.fromCeiling_mm = wall.maxZ * 1000 - f.corners[3].wallZ_mm;
        f.fromLeft_mm = f.corners[0].wallX_mm - wall.minX * 1000;
        f.fromRight_mm = wall.maxX * 1000 - f.corners[1].wallX_mm;
        
        // Confidence based on region size and coherence
        double expectedCells = (maxR - minR + 1) * (maxC - minC + 1);
        double fillRatio = region.size() / expectedCells;
        f.confidence = fillRatio * 0.9;  // Max 90% from shape
        
        return f;
    }
    
    // ========================================================================
    // CHECK FEATURE SIZE VALIDITY
    // ========================================================================
    
    bool isValidFeatureSize(const DetectedFeature& f) {
        // Check minimum size
        if (f.width_mm < cfg.minFeatureSize_mm || f.height_mm < cfg.minFeatureSize_mm) {
            return false;
        }
        
        // Check maximum size
        if (f.width_mm > cfg.maxFeatureSize_mm || f.height_mm > cfg.maxFeatureSize_mm) {
            return false;
        }
        
        // Feature must be smaller than wall (with margin)
        double margin = 100;  // 100mm margin
        if (f.width_mm >= wall.width_mm - margin || f.height_mm >= wall.height_mm - margin) {
            return false;
        }
        
        // Feature must be inside wall bounds
        if (f.fromFloor_mm < -50 || f.fromCeiling_mm < -50 || 
            f.fromLeft_mm < -50 || f.fromRight_mm < -50) {
            return false;
        }
        
        return true;
    }
    
    // ========================================================================
    // MAIN SCAN PIPELINE
    // ========================================================================
    
    WallScanResult scan(UnitreeLidarReader* reader) {
        WallScanResult result;
        auto t0 = std::chrono::steady_clock::now();
        
        if (!collect(reader)) {
            std::cout << "  ERROR: Not enough data!\n";
            return result;
        }
        
        // STOP LIDAR
        if (cfg.verbose) std::cout << "\n  Stopping LiDAR...\n";
        reader->stopLidarRotation();
        
        if (!isRunning()) return result;
        
        result.totalPoints = allPoints.size();
        
        if (!detectWallPlane()) return result;
        result.wallPoints = wallPoints.size();
        
        if (!computeWallBoundaries()) return result;
        result.wall = wall;
        
        if (!buildDepthMap()) return result;
        
        detectEdges();
        
        result.features = findFeatures();
        
        // Sort features by position (top-left to bottom-right)
        std::sort(result.features.begin(), result.features.end(),
            [](const DetectedFeature& a, const DetectedFeature& b) {
                if (std::abs(a.corners[0].wallZ_mm - b.corners[0].wallZ_mm) > 200) {
                    return a.corners[0].wallZ_mm > b.corners[0].wallZ_mm;  // Higher first
                }
                return a.corners[0].wallX_mm < b.corners[0].wallX_mm;  // Left first
            });
        
        auto t1 = std::chrono::steady_clock::now();
        result.scanSeconds = std::chrono::duration<double>(t1 - t0).count();
        result.valid = true;
        
        if (cfg.verbose) {
            std::cout << "\n  [6] Done! Features: " << result.features.size() << "\n";
        }
        
        return result;
    }
    
    // ========================================================================
    // OUTPUT - CORNER FOCUSED
    // ========================================================================
    
    void printResults(const WallScanResult& result) {
        if (!result.valid) {
            std::cout << "\n  *** SCAN FAILED ***\n";
            return;
        }
        
        std::cout << "\n";
        std::cout << "================================================================\n";
        std::cout << "            WALL CORNER DETECTION RESULTS                      \n";
        std::cout << "================================================================\n";
        
        // Wall info
        std::cout << "\n  WALL\n";
        std::cout << "  ----\n";
        std::cout << std::fixed << std::setprecision(0);
        std::cout << "    Size: " << result.wall.width_mm << " x " << result.wall.height_mm << " mm\n";
        std::cout << "    (Width x Height)\n\n";
        
        // Wall corners
        std::cout << "    WALL CORNERS (mm):\n";
        std::cout << "    ┌────────┬─────────────────────────────────────────────────┐\n";
        std::cout << "    │ Corner │      X          Y          Z      (Wall X, Z)   │\n";
        std::cout << "    ├────────┼─────────────────────────────────────────────────┤\n";
        const char* cornerNames[] = {"BL", "BR", "TR", "TL"};
        for (int i = 0; i < 4; i++) {
            const Corner& c = result.wall.corners[i];
            std::cout << "    │   " << cornerNames[i] << "   │ " 
                      << std::setw(8) << std::setprecision(1) << c.pos_mm.x << "  "
                      << std::setw(8) << c.pos_mm.y << "  "
                      << std::setw(8) << c.pos_mm.z << "  ("
                      << std::setw(6) << c.wallX_mm << ", "
                      << std::setw(6) << c.wallZ_mm << ") │\n";
        }
        std::cout << "    └────────┴─────────────────────────────────────────────────┘\n";
        
        // Features
        std::cout << "\n  DETECTED FEATURES: " << result.features.size() << "\n";
        std::cout << "  ------------------\n";
        
        if (result.features.empty()) {
            std::cout << "    (No features detected)\n";
        }
        
        for (size_t fi = 0; fi < result.features.size(); fi++) {
            const DetectedFeature& f = result.features[fi];
            
            std::cout << "\n  ┌─────────────────────────────────────────────────────────┐\n";
            std::cout << "  │ FEATURE #" << (fi + 1) << " - " << featureTypeName(f.type);
            int padding = 42 - strlen(featureTypeName(f.type));
            for (int p = 0; p < padding; p++) std::cout << " ";
            std::cout << "│\n";
            std::cout << "  ├─────────────────────────────────────────────────────────┤\n";
            
            // Corners
            std::cout << "  │ CORNERS (mm):                                           │\n";
            for (int i = 0; i < 4; i++) {
                const Corner& c = f.corners[i];
                std::cout << "  │   " << cornerNames[i] << ": X=" << std::setw(7) << std::setprecision(0) << c.pos_mm.x
                          << "  Y=" << std::setw(7) << c.pos_mm.y
                          << "  Z=" << std::setw(7) << c.pos_mm.z << "        │\n";
            }
            
            std::cout << "  ├─────────────────────────────────────────────────────────┤\n";
            
            // Derived dimensions
            std::cout << "  │ DERIVED DIMENSIONS:                                     │\n";
            std::cout << "  │   Width:  " << std::setw(6) << f.width_mm << " mm   (BR.x - BL.x)                   │\n";
            std::cout << "  │   Height: " << std::setw(6) << f.height_mm << " mm   (TL.z - BL.z)                   │\n";
            if (f.type == FeatureType::PROTRUSION) {
                std::cout << "  │   Depth:  " << std::setw(6) << f.depth_mm << " mm   (protrusion from wall)         │\n";
            }
            
            std::cout << "  ├─────────────────────────────────────────────────────────┤\n";
            
            // Position
            std::cout << "  │ POSITION ON WALL:                                       │\n";
            std::cout << "  │   From floor:   " << std::setw(6) << f.fromFloor_mm << " mm                            │\n";
            std::cout << "  │   From ceiling: " << std::setw(6) << f.fromCeiling_mm << " mm                            │\n";
            std::cout << "  │   From left:    " << std::setw(6) << f.fromLeft_mm << " mm                            │\n";
            std::cout << "  │   From right:   " << std::setw(6) << f.fromRight_mm << " mm                            │\n";
            
            std::cout << "  ├─────────────────────────────────────────────────────────┤\n";
            std::cout << "  │ Confidence: " << std::setw(3) << (int)(f.confidence * 100) << "%                                        │\n";
            std::cout << "  └─────────────────────────────────────────────────────────┘\n";
        }
        
        // Summary table
        if (!result.features.empty()) {
            std::cout << "\n  SUMMARY TABLE (all in mm)\n";
            std::cout << "  ─────────────────────────────────────────────────────────────────────────\n";
            std::cout << "  │ # │ Type │ Width │Height│ BL Corner (X,Y,Z)    │ TR Corner (X,Y,Z)    │\n";
            std::cout << "  ─────────────────────────────────────────────────────────────────────────\n";
            
            for (size_t fi = 0; fi < result.features.size(); fi++) {
                const DetectedFeature& f = result.features[fi];
                std::cout << "  │" << std::setw(2) << (fi+1) << " │ "
                          << std::setw(4) << (f.type == FeatureType::HOLE ? "HOLE" : "PROT") << " │"
                          << std::setw(6) << std::setprecision(0) << f.width_mm << " │"
                          << std::setw(5) << f.height_mm << " │"
                          << "(" << std::setw(5) << f.corners[0].pos_mm.x << ","
                          << std::setw(5) << f.corners[0].pos_mm.y << ","
                          << std::setw(5) << f.corners[0].pos_mm.z << ") │"
                          << "(" << std::setw(5) << f.corners[2].pos_mm.x << ","
                          << std::setw(5) << f.corners[2].pos_mm.y << ","
                          << std::setw(5) << f.corners[2].pos_mm.z << ") │\n";
            }
            std::cout << "  ─────────────────────────────────────────────────────────────────────────\n";
        }
        
        // Stats
        std::cout << "\n  STATISTICS\n";
        std::cout << "  ----------\n";
        std::cout << "    Total points: " << result.totalPoints << "\n";
        std::cout << "    Wall points:  " << result.wallPoints << "\n";
        std::cout << "    Scan time:    " << std::setprecision(1) << result.scanSeconds << " sec\n";
        
        std::cout << "\n================================================================\n";
    }
};

#endif // WALL_FEATURE_DETECTOR_H