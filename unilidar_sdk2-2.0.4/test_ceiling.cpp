/**********************************************************************
 * TEST FILE - Simulates LiDAR data without hardware
 * 
 * Compile: g++ -o test_ceiling test_ceiling.cpp -std=c++11
 * Run:     ./test_ceiling
 ***********************************************************************/

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <cstdlib>

struct Point3D {
    double x, y, z;
    Point3D(double _x=0, double _y=0, double _z=0) : x(_x), y(_y), z(_z) {}
    double distanceXY(const Point3D& o) const {
        return std::sqrt((x-o.x)*(x-o.x) + (y-o.y)*(y-o.y));
    }
};

struct Point2D {
    double x, y;
    Point2D(double _x=0, double _y=0) : x(_x), y(_y) {}
};

// Generate fake ceiling points for a room
std::vector<Point3D> generateFakeCeiling(double length_m, double width_m, double height_m) {
    std::vector<Point3D> points;
    srand(42);
    
    // Generate ceiling points
    for (int i = 0; i < 10000; i++) {
        double x = ((rand() % 1000) / 1000.0 - 0.5) * length_m;
        double y = ((rand() % 1000) / 1000.0 - 0.5) * width_m;
        double z = height_m + ((rand() % 100) / 1000.0 - 0.05); // ±5cm noise
        points.push_back(Point3D(x, y, z));
    }
    
    // Add some floor/wall points (noise)
    for (int i = 0; i < 2000; i++) {
        double x = ((rand() % 1000) / 1000.0 - 0.5) * length_m;
        double y = ((rand() % 1000) / 1000.0 - 0.5) * width_m;
        double z = (rand() % 1000) / 1000.0 * height_m * 0.5;
        points.push_back(Point3D(x, y, z));
    }
    
    // Add outliers (like your 6.5m reading)
    points.push_back(Point3D(0, 0, 6.5));
    points.push_back(Point3D(1, 1, 7.2));
    
    return points;
}

double cross2D(const Point2D& O, const Point2D& A, const Point2D& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

std::vector<Point2D> convexHull(std::vector<Point2D> pts) {
    int n = pts.size();
    if (n < 3) return pts;
    int minIdx = 0;
    for (int i = 1; i < n; i++)
        if (pts[i].y < pts[minIdx].y || (pts[i].y == pts[minIdx].y && pts[i].x < pts[minIdx].x))
            minIdx = i;
    std::swap(pts[0], pts[minIdx]);
    Point2D pivot = pts[0];
    std::sort(pts.begin()+1, pts.end(), [&](const Point2D& a, const Point2D& b) {
        double v = cross2D(pivot, a, b);
        if (std::abs(v) < 1e-10) {
            double da = (a.x-pivot.x)*(a.x-pivot.x)+(a.y-pivot.y)*(a.y-pivot.y);
            double db = (b.x-pivot.x)*(b.x-pivot.x)+(b.y-pivot.y)*(b.y-pivot.y);
            return da < db;
        }
        return v > 0;
    });
    std::vector<Point2D> hull;
    for (int i = 0; i < n; i++) {
        while (hull.size() > 1 && cross2D(hull[hull.size()-2], hull[hull.size()-1], pts[i]) <= 0)
            hull.pop_back();
        hull.push_back(pts[i]);
    }
    return hull;
}

void minAreaRect(const std::vector<Point2D>& hull, Point2D rect[4]) {
    int n = hull.size();
    double minArea = 1e18;
    for (int i = 0; i < n; i++) {
        Point2D edge;
        edge.x = hull[(i+1)%n].x - hull[i].x;
        edge.y = hull[(i+1)%n].y - hull[i].y;
        double len = std::sqrt(edge.x*edge.x + edge.y*edge.y);
        if (len < 1e-10) continue;
        edge.x /= len; edge.y /= len;
        Point2D perp = {-edge.y, edge.x};
        double minP=1e18, maxP=-1e18, minPe=1e18, maxPe=-1e18;
        for (auto& p : hull) {
            double proj = p.x*edge.x + p.y*edge.y;
            double perpP = p.x*perp.x + p.y*perp.y;
            minP = std::min(minP, proj); maxP = std::max(maxP, proj);
            minPe = std::min(minPe, perpP); maxPe = std::max(maxPe, perpP);
        }
        double area = (maxP-minP) * (maxPe-minPe);
        if (area < minArea) {
            minArea = area;
            rect[0] = {minP*edge.x + minPe*perp.x, minP*edge.y + minPe*perp.y};
            rect[1] = {maxP*edge.x + minPe*perp.x, maxP*edge.y + minPe*perp.y};
            rect[2] = {maxP*edge.x + maxPe*perp.x, maxP*edge.y + maxPe*perp.y};
            rect[3] = {minP*edge.x + maxPe*perp.x, minP*edge.y + maxPe*perp.y};
        }
    }
}

int main() {
    // SIMULATED ROOM: 4m x 3m, ceiling at 2.8m
    double TRUE_LENGTH = 4.0;
    double TRUE_WIDTH = 3.0;
    double TRUE_HEIGHT = 2.8;
    
    std::cout << "============================================\n";
    std::cout << "  CEILING DETECTOR TEST (No Hardware)\n";
    std::cout << "============================================\n";
    std::cout << "\nSimulated room: " << TRUE_LENGTH << "m x " << TRUE_WIDTH << "m\n";
    std::cout << "Ceiling height: " << TRUE_HEIGHT << "m\n\n";
    
    // Generate fake data
    auto points = generateFakeCeiling(TRUE_LENGTH, TRUE_WIDTH, TRUE_HEIGHT);
    std::cout << "Generated " << points.size() << " fake points\n";
    
    // Get Z values and find percentile
    std::vector<double> zVals;
    for (auto& p : points) zVals.push_back(p.z);
    std::sort(zVals.begin(), zVals.end());
    
    double maxZ = zVals.back();
    double p95 = zVals[(int)(0.95 * zVals.size())];
    
    std::cout << "\nZ range: " << zVals.front() << " to " << maxZ << " m\n";
    std::cout << "95th percentile: " << p95 << " m (used for ceiling)\n";
    
    // Filter ceiling points
    std::vector<Point3D> ceiling;
    double avgZ = 0;
    for (auto& p : points) {
        if (p.z >= p95 - 0.2 && p.z <= p95 + 0.2) {
            ceiling.push_back(p);
            avgZ += p.z;
        }
    }
    avgZ /= ceiling.size();
    std::cout << "Ceiling points: " << ceiling.size() << "\n";
    
    // Project to 2D and find corners
    std::vector<Point2D> pts2d;
    for (auto& p : ceiling) pts2d.push_back({p.x, p.y});
    
    auto hull = convexHull(pts2d);
    Point2D rect[4];
    minAreaRect(hull, rect);
    
    // Calculate measurements
    Point3D corners[4];
    for (int i = 0; i < 4; i++) corners[i] = {rect[i].x, rect[i].y, avgZ};
    
    double d01 = corners[0].distanceXY(corners[1]) * 1000;
    double d12 = corners[1].distanceXY(corners[2]) * 1000;
    double length = std::max(d01, d12);
    double width = std::min(d01, d12);
    double diag1 = corners[0].distanceXY(corners[2]) * 1000;
    double diag2 = corners[1].distanceXY(corners[3]) * 1000;
    
    // OUTPUT
    std::cout << "\n============================================\n";
    std::cout << "  RESULTS\n";
    std::cout << "============================================\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Ceiling Height: " << avgZ * 1000 << " mm (" << avgZ << " m)\n";
    std::cout << "Length:         " << length << " mm (" << length/1000 << " m)\n";
    std::cout << "Width:          " << width << " mm (" << width/1000 << " m)\n";
    std::cout << "Diagonal 1:     " << diag1 << " mm\n";
    std::cout << "Diagonal 2:     " << diag2 << " mm\n";
    std::cout << "Area:           " << std::setprecision(2) << (length/1000)*(width/1000) << " sq.m\n";
    
    std::cout << "\n--- Accuracy Check ---\n";
    std::cout << "Expected: " << TRUE_LENGTH*1000 << " x " << TRUE_WIDTH*1000 << " mm\n";
    std::cout << "Got:      " << length << " x " << width << " mm\n";
    std::cout << "Error:    " << std::abs(length - TRUE_LENGTH*1000) << " x " 
              << std::abs(width - TRUE_WIDTH*1000) << " mm\n";
    
    return 0;
}