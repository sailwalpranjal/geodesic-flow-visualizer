#include "../include/geometry.hpp"
#include "../include/surface.hpp"
#include "../include/renderer.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

// Compute first fundamental form (metric tensor) at parameter (u, v)
FirstFundamentalForm computeFirstFundamentalForm(const Surface& surface, double u, double v) {
    Vec3 r_u = surface.eval_du(u, v);
    Vec3 r_v = surface.eval_dv(u, v);
    
    FirstFundamentalForm form;
    form.E = r_u.dot(r_u);
    form.F = r_u.dot(r_v);
    form.G = r_v.dot(r_v);
    
    return form;
}

// Compute second fundamental form (curvature tensor) at parameter (u, v)
SecondFundamentalForm computeSecondFundamentalForm(const Surface& surface, double u, double v) {
    Vec3 r_u = surface.eval_du(u, v);
    Vec3 r_v = surface.eval_dv(u, v);
    Vec3 r_uu = surface.eval_duu(u, v);
    Vec3 r_uv = surface.eval_duv(u, v);
    Vec3 r_vv = surface.eval_dvv(u, v);
    
    Vec3 normal = r_u.cross(r_v).normalized();
    
    SecondFundamentalForm form;
    form.L = r_uu.dot(normal);
    form.M = r_uv.dot(normal);
    form.N = r_vv.dot(normal);
    
    return form;
}

// Compute Gaussian and mean curvature at parameter (u, v)
CurvatureInfo computeCurvature(const Surface& surface, double u, double v) {
    FirstFundamentalForm first = computeFirstFundamentalForm(surface, u, v);
    SecondFundamentalForm second = computeSecondFundamentalForm(surface, u, v);
    
    CurvatureInfo info;
    info.normal = surface.normal(u, v);
    
    double det1 = first.determinant();
    if (std::abs(det1) < EPSILON) {
        info.gaussian = 0.0;
        info.mean = 0.0;
        return info;
    }
    
    // Gaussian curvature: K = (LN - M²)/(EG - F²)
    double numeratorK = second.L * second.N - second.M * second.M;
    info.gaussian = numeratorK / det1;
    
    // Mean curvature: H = (EN - 2FM + GL)/(2(EG - F²))
    double numeratorH = first.E * second.N - 2.0 * first.F * second.M + first.G * second.L;
    info.mean = numeratorH / (2.0 * det1);
    
    return info;
}

// Compute Christoffel symbols of the second kind at parameter (u, v)
ChristoffelSymbols computeChristoffelSymbols(const Surface& surface, double u, double v) {
    // Compute metric tensor and its derivatives using finite differences
    const double h = 1e-6;
    
    FirstFundamentalForm g = computeFirstFundamentalForm(surface, u, v);
    FirstFundamentalForm g_u = computeFirstFundamentalForm(surface, u + h, v);
    FirstFundamentalForm g_v = computeFirstFundamentalForm(surface, u, v + h);
    
    // Derivatives of metric tensor components
    double E_u = (g_u.E - g.E) / h;
    double E_v = (g_v.E - g.E) / h;
    double F_u = (g_u.F - g.F) / h;
    double F_v = (g_v.F - g.F) / h;
    double G_u = (g_u.G - g.G) / h;
    double G_v = (g_v.G - g.G) / h;
    
    // Inverse metric tensor
    double g_inv_11 = g.E_inv();
    double g_inv_12 = g.F_inv();
    double g_inv_22 = g.G_inv();
    
    ChristoffelSymbols symbols;
    
    // Γ^1_11 = g^{11}(E_u)/2 + g^{12}(E_v - F_u)
    symbols.gamma1_11 = 0.5 * g_inv_11 * E_u + g_inv_12 * (E_v - F_u);
    
    // Γ^1_12 = Γ^1_21 = g^{11}(E_v)/2 + g^{12}(G_u)/2
    symbols.gamma1_12 = 0.5 * g_inv_11 * E_v + 0.5 * g_inv_12 * G_u;
    symbols.gamma1_21 = symbols.gamma1_12;
    
    // Γ^1_22 = g^{11}(F_v - G_u/2) + g^{12}(G_v)/2
    symbols.gamma1_22 = g_inv_11 * (F_v - 0.5 * G_u) + 0.5 * g_inv_12 * G_v;
    
    // Γ^2_11 = g^{12}(E_u)/2 + g^{22}(E_v - F_u)
    symbols.gamma2_11 = 0.5 * g_inv_12 * E_u + g_inv_22 * (E_v - F_u);
    
    // Γ^2_12 = Γ^2_21 = g^{12}(E_v)/2 + g^{22}(G_u)/2
    symbols.gamma2_12 = 0.5 * g_inv_12 * E_v + 0.5 * g_inv_22 * G_u;
    symbols.gamma2_21 = symbols.gamma2_12;
    
    // Γ^2_22 = g^{12}(F_v - G_u/2) + g^{22}(G_v)/2
    symbols.gamma2_22 = g_inv_12 * (F_v - 0.5 * G_u) + 0.5 * g_inv_22 * G_v;
    
    return symbols;
}

// Geodesic equation right-hand side: computes derivatives for RK4 integration
// State = (u, v, u', v') where ' denotes derivative with respect to parameter t
// Returns (u', v', u'', v'')
void geodesicDerivatives(const Surface& surface, const GeodesicState& state, 
                         double& du_dt, double& dv_dt, double& d2u_dt2, double& d2v_dt2) {
    double u = state.position.u;
    double v = state.position.v;
    double u_dot = state.velocity.u;
    double v_dot = state.velocity.v;
    
    // Clamp parameters to valid range
    double u_clamped = std::max(surface.uMin(), std::min(surface.uMax(), u));
    double v_clamped = std::max(surface.vMin(), std::min(surface.vMax(), v));
    
    ChristoffelSymbols gamma = computeChristoffelSymbols(surface, u_clamped, v_clamped);
    
    // Geodesic equation: d²u/dt² = -Γ^1_ij (du^i/dt)(du^j/dt)
    //                    d²v/dt² = -Γ^2_ij (du^i/dt)(du^j/dt)
    
    du_dt = u_dot;
    dv_dt = v_dot;
    
    d2u_dt2 = -(gamma.gamma1_11 * u_dot * u_dot + 
                2.0 * gamma.gamma1_12 * u_dot * v_dot + 
                gamma.gamma1_22 * v_dot * v_dot);
    
    d2v_dt2 = -(gamma.gamma2_11 * u_dot * u_dot + 
                2.0 * gamma.gamma2_12 * u_dot * v_dot + 
                gamma.gamma2_22 * v_dot * v_dot);
}

// Fourth-order Runge-Kutta step for geodesic integration
GeodesicState rk4Step(const Surface& surface, const GeodesicState& state, double dt) {
    double u = state.position.u;
    double v = state.position.v;
    double u_dot = state.velocity.u;
    double v_dot = state.velocity.v;
    
    // k1
    double k1_u, k1_v, k1_u_dot, k1_v_dot;
    geodesicDerivatives(surface, state, k1_u, k1_v, k1_u_dot, k1_v_dot);
    
    // k2
    GeodesicState state2;
    state2.position.u = u + 0.5 * dt * k1_u;
    state2.position.v = v + 0.5 * dt * k1_v;
    state2.velocity.u = u_dot + 0.5 * dt * k1_u_dot;
    state2.velocity.v = v_dot + 0.5 * dt * k1_v_dot;
    double k2_u, k2_v, k2_u_dot, k2_v_dot;
    geodesicDerivatives(surface, state2, k2_u, k2_v, k2_u_dot, k2_v_dot);
    
    // k3
    GeodesicState state3;
    state3.position.u = u + 0.5 * dt * k2_u;
    state3.position.v = v + 0.5 * dt * k2_v;
    state3.velocity.u = u_dot + 0.5 * dt * k2_u_dot;
    state3.velocity.v = v_dot + 0.5 * dt * k2_v_dot;
    double k3_u, k3_v, k3_u_dot, k3_v_dot;
    geodesicDerivatives(surface, state3, k3_u, k3_v, k3_u_dot, k3_v_dot);
    
    // k4
    GeodesicState state4;
    state4.position.u = u + dt * k3_u;
    state4.position.v = v + dt * k3_v;
    state4.velocity.u = u_dot + dt * k3_u_dot;
    state4.velocity.v = v_dot + dt * k3_v_dot;
    double k4_u, k4_v, k4_u_dot, k4_v_dot;
    geodesicDerivatives(surface, state4, k4_u, k4_v, k4_u_dot, k4_v_dot);
    
    // Combine using RK4 weights
    GeodesicState result;
    result.position.u = u + (dt / 6.0) * (k1_u + 2.0 * k2_u + 2.0 * k3_u + k4_u);
    result.position.v = v + (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
    result.velocity.u = u_dot + (dt / 6.0) * (k1_u_dot + 2.0 * k2_u_dot + 2.0 * k3_u_dot + k4_u_dot);
    result.velocity.v = v_dot + (dt / 6.0) * (k1_v_dot + 2.0 * k2_v_dot + 2.0 * k3_v_dot + k4_v_dot);
    
    return result;
}

// Compute adaptive step size based on local curvature
double adaptiveStepSize(const Surface& surface, double u, double v, double baseStepSize) {
    CurvatureInfo curv = computeCurvature(surface, u, v);
    
    // Adjust step size based on curvature magnitude
    double curvatureMagnitude = std::abs(curv.gaussian);
    double adaptiveFactor = 1.0;
    
    if (curvatureMagnitude > 1.0) {
        adaptiveFactor = 0.5;
    } else if (curvatureMagnitude > 0.1) {
        adaptiveFactor = 0.8;
    }
    
    return baseStepSize * adaptiveFactor;
}

// Integrate geodesic equation from initial state
GeodesicPath integrateGeodesic(const Surface& surface, const GeodesicState& initialState, 
                               double totalTime, double baseStepSize, int maxSteps) {
    GeodesicPath path;
    GeodesicState state = initialState;
    
    double t = 0.0;
    int steps = 0;
    
    while (t < totalTime && steps < maxSteps) {
        // Add current point to path
        Vec3 point3D = surface.eval(state.position.u, state.position.v);
        path.addPoint(point3D, t);
        
        // Compute adaptive step size
        double dt = adaptiveStepSize(surface, state.position.u, state.position.v, baseStepSize);
        dt = std::min(dt, totalTime - t);
        
        // Perform RK4 step
        state = rk4Step(surface, state, dt);
        
        // Normalize velocity to maintain unit speed
        double velocityMag = state.velocity.length();
        if (velocityMag > EPSILON) {
            state.velocity = state.velocity / velocityMag;
        }
        
        // Check if we've left valid parameter range
        if (!surface.isValidParameter(state.position.u, state.position.v)) {
            break;
        }
        
        t += dt;
        steps++;
    }
    
    return path;
}

// Generate mesh vertices for surface visualization
std::vector<MeshVertex> generateSurfaceMesh(const Surface& surface, const MeshParameters& params) {
    std::vector<MeshVertex> vertices;
    vertices.reserve(params.uResolution * params.vResolution);
    
    double uMin = surface.uMin();
    double uMax = surface.uMax();
    double vMin = surface.vMin();
    double vMax = surface.vMax();
    
    double du = (uMax - uMin) / (params.uResolution - 1);
    double dv = (vMax - vMin) / (params.vResolution - 1);
    
    for (int i = 0; i < params.vResolution; ++i) {
        double v = vMin + i * dv;
        for (int j = 0; j < params.uResolution; ++j) {
            double u = uMin + j * du;
            
            Vec3 pos = surface.eval(u, v);
            Vec3 normal = surface.normal(u, v);
            
            float curvature = 0.0f;
            if (params.computeCurvature) {
                CurvatureInfo curv = computeCurvature(surface, u, v);
                curvature = static_cast<float>(curv.gaussian);
            }
            
            vertices.emplace_back(pos, normal, curvature);
        }
    }
    
    return vertices;
}

// Generate triangle indices for mesh rendering
std::vector<unsigned int> generateMeshIndices(const MeshParameters& params) {
    std::vector<unsigned int> indices;
    int uRes = params.uResolution;
    int vRes = params.vResolution;
    
    indices.reserve((uRes - 1) * (vRes - 1) * 6);
    
    for (int i = 0; i < vRes - 1; ++i) {
        for (int j = 0; j < uRes - 1; ++j) {
            unsigned int topLeft = i * uRes + j;
            unsigned int topRight = topLeft + 1;
            unsigned int bottomLeft = (i + 1) * uRes + j;
            unsigned int bottomRight = bottomLeft + 1;
            
            // First triangle
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);
            
            // Second triangle
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }
    
    return indices;
}

// Find closest point on surface to a given 3D position (for mouse picking)
Vec2 findClosestParameterPoint(const Surface& surface, const Vec3& targetPoint, 
                                const Vec2& initialGuess, int maxIterations) {
    Vec2 current = initialGuess;
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        Vec3 surfacePoint = surface.eval(current.u, current.v);
        Vec3 diff = targetPoint - surfacePoint;
        
        if (diff.length() < EPSILON) {
            break;
        }
        
        // Compute Jacobian
        Vec3 r_u = surface.eval_du(current.u, current.v);
        Vec3 r_v = surface.eval_dv(current.v, current.v);
        
        // Gradient descent step
        double step_u = diff.dot(r_u) * 0.1;
        double step_v = diff.dot(r_v) * 0.1;
        
        current.u += step_u;
        current.v += step_v;
        
        // Clamp to valid range
        current.u = std::max(surface.uMin(), std::min(surface.uMax(), current.u));
        current.v = std::max(surface.vMin(), std::min(surface.vMax(), current.v));
    }
    
    return current;
}