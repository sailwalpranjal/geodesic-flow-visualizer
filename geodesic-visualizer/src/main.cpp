#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "../include/geometry.hpp"
#include "../include/surface.hpp"
#include "../include/renderer.hpp"
#include <memory>
#include <vector>
#include <string>
#include <emscripten.h>

using namespace emscripten;

// Global application state
static std::unique_ptr<Surface> currentSurface;
static std::vector<GeodesicPath> geodesicPaths;
static MeshParameters meshParams(100, 100, true);
static Camera camera;
static RenderState renderState;
static PerformanceMetrics metrics;

// External declarations for functions defined in geometry_core.cpp
extern FirstFundamentalForm computeFirstFundamentalForm(const Surface& surface, double u, double v);
extern SecondFundamentalForm computeSecondFundamentalForm(const Surface& surface, double u, double v);
extern CurvatureInfo computeCurvature(const Surface& surface, double u, double v);
extern ChristoffelSymbols computeChristoffelSymbols(const Surface& surface, double u, double v);
extern GeodesicPath integrateGeodesic(const Surface& surface, const GeodesicState& initialState, 
                                      double totalTime, double baseStepSize, int maxSteps);
extern std::vector<MeshVertex> generateSurfaceMesh(const Surface& surface, const MeshParameters& params);
extern std::vector<unsigned int> generateMeshIndices(const MeshParameters& params);
extern Vec2 findClosestParameterPoint(const Surface& surface, const Vec3& targetPoint, 
                                      const Vec2& initialGuess, int maxIterations);

// Initialize the application with a specific surface type
void initialize(const std::string& surfaceType) {
    currentSurface = createSurface(surfaceType);
    geodesicPaths.clear();
    
    // Set default camera position
    camera.position = Vec3(0, 0, 8);
    camera.target = Vec3(0, 0, 0);
    camera.up = Vec3(0, 1, 0);
    camera.fov = 45.0;
    camera.aspect = 1.0;
    camera.near = 0.1;
    camera.far = 100.0;
}

// Change the current surface type
void setSurface(const std::string& surfaceType) {
    currentSurface = createSurface(surfaceType);
    geodesicPaths.clear();
}

// Generate mesh data for the current surface
val generateMesh() {
    if (!currentSurface) {
        return val::object();
    }
    
    auto startTime = emscripten_get_now();
    
    std::vector<MeshVertex> vertices = generateSurfaceMesh(*currentSurface, meshParams);
    std::vector<unsigned int> indices = generateMeshIndices(meshParams);
    
    auto endTime = emscripten_get_now();
    metrics.meshGenerationTime = endTime - startTime;
    metrics.vertexCount = vertices.size();
    metrics.triangleCount = indices.size() / 3;
    
    // Create JavaScript object with mesh data
    val result = val::object();
    
    // Convert vertices to Float32Array
    val vertexArray = val::global("Float32Array").new_(vertices.size() * 7);
    for (size_t i = 0; i < vertices.size(); ++i) {
        vertexArray.call<void>("set", 
            val::array(std::vector<float>{
                vertices[i].x, vertices[i].y, vertices[i].z,
                vertices[i].nx, vertices[i].ny, vertices[i].nz,
                vertices[i].curvature
            }), i * 7);
    }
    
    // Convert indices to Uint32Array
    val indexArray = val::global("Uint32Array").new_(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indexArray.set(i, indices[i]);
    }
    
    result.set("vertices", vertexArray);
    result.set("indices", indexArray);
    result.set("vertexCount", vertices.size());
    result.set("triangleCount", indices.size() / 3);
    
    return result;
}

// Compute geodesic from starting point and direction
val computeGeodesic(double u0, double v0, double du0, double dv0, 
                    double totalTime, double stepSize, int maxSteps) {
    if (!currentSurface) {
        return val::array();
    }
    
    auto startTime = emscripten_get_now();
    
    // Normalize initial direction
    Vec2 direction(du0, dv0);
    double dirMag = direction.length();
    if (dirMag > EPSILON) {
        direction = direction / dirMag;
    } else {
        direction = Vec2(1.0, 0.0);
    }
    
    GeodesicState initialState;
    initialState.position = Vec2(u0, v0);
    initialState.velocity = direction;
    
    GeodesicPath path = integrateGeodesic(*currentSurface, initialState, 
                                          totalTime, stepSize, maxSteps);
    
    auto endTime = emscripten_get_now();
    metrics.geodesicComputationTime = endTime - startTime;
    
    // Convert path to JavaScript array
    val result = val::array();
    for (size_t i = 0; i < path.points.size(); ++i) {
        val point = val::object();
        point.set("x", path.points[i].x);
        point.set("y", path.points[i].y);
        point.set("z", path.points[i].z);
        point.set("t", path.parameters[i]);
        result.call<void>("push", point);
    }
    
    return result;
}

// Get curvature information at a specific point on the surface
val getCurvatureAt(double u, double v) {
    if (!currentSurface) {
        return val::object();
    }
    
    CurvatureInfo curv = computeCurvature(*currentSurface, u, v);
    
    val result = val::object();
    result.set("gaussian", curv.gaussian);
    result.set("mean", curv.mean);
    result.set("normalX", curv.normal.x);
    result.set("normalY", curv.normal.y);
    result.set("normalZ", curv.normal.z);
    
    return result;
}

// Get first fundamental form at a point
val getFirstFundamentalForm(double u, double v) {
    if (!currentSurface) {
        return val::object();
    }
    
    FirstFundamentalForm form = computeFirstFundamentalForm(*currentSurface, u, v);
    
    val result = val::object();
    result.set("E", form.E);
    result.set("F", form.F);
    result.set("G", form.G);
    result.set("determinant", form.determinant());
    
    return result;
}

// Get second fundamental form at a point
val getSecondFundamentalForm(double u, double v) {
    if (!currentSurface) {
        return val::object();
    }
    
    SecondFundamentalForm form = computeSecondFundamentalForm(*currentSurface, u, v);
    
    val result = val::object();
    result.set("L", form.L);
    result.set("M", form.M);
    result.set("N", form.N);
    
    return result;
}

// Get Christoffel symbols at a point
val getChristoffelSymbols(double u, double v) {
    if (!currentSurface) {
        return val::object();
    }
    
    ChristoffelSymbols symbols = computeChristoffelSymbols(*currentSurface, u, v);
    
    val result = val::object();
    result.set("gamma1_11", symbols.gamma1_11);
    result.set("gamma1_12", symbols.gamma1_12);
    result.set("gamma1_21", symbols.gamma1_21);
    result.set("gamma1_22", symbols.gamma1_22);
    result.set("gamma2_11", symbols.gamma2_11);
    result.set("gamma2_12", symbols.gamma2_12);
    result.set("gamma2_21", symbols.gamma2_21);
    result.set("gamma2_22", symbols.gamma2_22);
    
    return result;
}

// Evaluate surface at parameter point
val evaluateSurface(double u, double v) {
    if (!currentSurface) {
        return val::object();
    }
    
    Vec3 point = currentSurface->eval(u, v);
    
    val result = val::object();
    result.set("x", point.x);
    result.set("y", point.y);
    result.set("z", point.z);
    
    return result;
}

// Get surface parameter bounds
val getSurfaceBounds() {
    if (!currentSurface) {
        return val::object();
    }
    
    val result = val::object();
    result.set("uMin", currentSurface->uMin());
    result.set("uMax", currentSurface->uMax());
    result.set("vMin", currentSurface->vMin());
    result.set("vMax", currentSurface->vMax());
    
    return result;
}

// Set mesh resolution
void setMeshResolution(int uRes, int vRes) {
    meshParams.uResolution = std::max(10, std::min(200, uRes));
    meshParams.vResolution = std::max(10, std::min(200, vRes));
}

// Get current performance metrics
val getPerformanceMetrics() {
    val result = val::object();
    result.set("meshGenerationTime", metrics.meshGenerationTime);
    result.set("geodesicComputationTime", metrics.geodesicComputationTime);
    result.set("renderTime", metrics.renderTime);
    result.set("frameRate", metrics.frameRate);
    result.set("vertexCount", metrics.vertexCount);
    result.set("triangleCount", metrics.triangleCount);
    
    return result;
}

// Update frame rate metric (called from JavaScript)
void updateFrameRate(double fps) {
    metrics.frameRate = fps;
}

// Ray-surface intersection for mouse picking
val rayIntersectSurface(double rayOriginX, double rayOriginY, double rayOriginZ,
                        double rayDirX, double rayDirY, double rayDirZ) {
    if (!currentSurface) {
        return val::null();
    }
    
    Vec3 rayOrigin(rayOriginX, rayOriginY, rayOriginZ);
    Vec3 rayDir(rayDirX, rayDirY, rayDirZ);
    rayDir.normalize();
    
    // Use grid search to find approximate intersection
    double bestDist = 1e10;
    double bestU = 0, bestV = 0;
    bool found = false;
    
    double uMin = currentSurface->uMin();
    double uMax = currentSurface->uMax();
    double vMin = currentSurface->vMin();
    double vMax = currentSurface->vMax();
    
    int gridSize = 50;
    double du = (uMax - uMin) / gridSize;
    double dv = (vMax - vMin) / gridSize;
    
    for (int i = 0; i < gridSize; ++i) {
        double v = vMin + i * dv;
        for (int j = 0; j < gridSize; ++j) {
            double u = uMin + j * du;
            
            Vec3 surfacePoint = currentSurface->eval(u, v);
            Vec3 toPoint = surfacePoint - rayOrigin;
            double proj = toPoint.dot(rayDir);
            
            if (proj > 0) {
                Vec3 closest = rayOrigin + rayDir * proj;
                double dist = (closest - surfacePoint).length();
                
                if (dist < bestDist) {
                    bestDist = dist;
                    bestU = u;
                    bestV = v;
                    found = true;
                }
            }
        }
    }
    
    if (!found || bestDist > 0.5) {
        return val::null();
    }
    
    // Refine using Newton's method
    Vec2 refined = findClosestParameterPoint(*currentSurface, 
                                             rayOrigin + rayDir * bestDist,
                                             Vec2(bestU, bestV), 10);
    
    Vec3 finalPoint = currentSurface->eval(refined.u, refined.v);
    
    val result = val::object();
    result.set("u", refined.u);
    result.set("v", refined.v);
    result.set("x", finalPoint.x);
    result.set("y", finalPoint.y);
    result.set("z", finalPoint.z);
    
    return result;
}

// Bindings for JavaScript access
EMSCRIPTEN_BINDINGS(geodesic_module) {
    function("initialize", &initialize);
    function("setSurface", &setSurface);
    function("generateMesh", &generateMesh);
    function("computeGeodesic", &computeGeodesic);
    function("getCurvatureAt", &getCurvatureAt);
    function("getFirstFundamentalForm", &getFirstFundamentalForm);
    function("getSecondFundamentalForm", &getSecondFundamentalForm);
    function("getChristoffelSymbols", &getChristoffelSymbols);
    function("evaluateSurface", &evaluateSurface);
    function("getSurfaceBounds", &getSurfaceBounds);
    function("setMeshResolution", &setMeshResolution);
    function("getPerformanceMetrics", &getPerformanceMetrics);
    function("updateFrameRate", &updateFrameRate);
    function("rayIntersectSurface", &rayIntersectSurface);
}