#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "geometry.hpp"
#include "surface.hpp"
#include <vector>

// Mesh vertex data structure for WebGL rendering
struct MeshVertex {
    float x, y, z;        // Position
    float nx, ny, nz;     // Normal
    float curvature;      // Gaussian curvature for color mapping
    
    MeshVertex() : x(0), y(0), z(0), nx(0), ny(1), nz(0), curvature(0) {}
    MeshVertex(const Vec3& pos, const Vec3& norm, float curv) 
        : x(pos.x), y(pos.y), z(pos.z), 
          nx(norm.x), ny(norm.y), nz(norm.z),
          curvature(curv) {}
};

// Mesh generation parameters
struct MeshParameters {
    int uResolution;
    int vResolution;
    bool computeCurvature;
    
    MeshParameters() 
        : uResolution(100), vResolution(100), computeCurvature(true) {}
    MeshParameters(int uRes, int vRes, bool curv = true)
        : uResolution(uRes), vResolution(vRes), computeCurvature(curv) {}
};

// Camera parameters for rendering
struct Camera {
    Vec3 position;
    Vec3 target;
    Vec3 up;
    double fov;
    double aspect;
    double near;
    double far;
    
    Camera() 
        : position(0, 0, 8), target(0, 0, 0), up(0, 1, 0),
          fov(45.0), aspect(1.0), near(0.1), far(100.0) {}
};

// Geodesic path data for rendering
struct GeodesicPath {
    std::vector<Vec3> points;
    std::vector<double> parameters; // t values
    double totalLength;
    
    GeodesicPath() : totalLength(0.0) {}
    
    void addPoint(const Vec3& point, double t) {
        points.push_back(point);
        parameters.push_back(t);
        
        if (points.size() > 1) {
            Vec3 diff = point - points[points.size() - 2];
            totalLength += diff.length();
        }
    }
    
    void clear() {
        points.clear();
        parameters.clear();
        totalLength = 0.0;
    }
    
    size_t size() const {
        return points.size();
    }
    
    bool empty() const {
        return points.empty();
    }
};

// Render state management
struct RenderState {
    bool showCurvature;
    bool showGeodesics;
    bool showNormals;
    double geodesicLineWidth;
    Vec3 backgroundColor;
    Vec3 lightDirection;
    
    RenderState()
        : showCurvature(true), showGeodesics(true), showNormals(false),
          geodesicLineWidth(2.0), 
          backgroundColor(0.1, 0.1, 0.15),
          lightDirection(Vec3(1, 1, 1).normalized()) {}
};

// Performance metrics
struct PerformanceMetrics {
    double meshGenerationTime;
    double geodesicComputationTime;
    double renderTime;
    double frameRate;
    int vertexCount;
    int triangleCount;
    
    PerformanceMetrics()
        : meshGenerationTime(0), geodesicComputationTime(0),
          renderTime(0), frameRate(0), vertexCount(0), triangleCount(0) {}
};

#endif // RENDERER_HPP