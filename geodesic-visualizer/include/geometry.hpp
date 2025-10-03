#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>
#include <algorithm>

constexpr double EPSILON = 1e-10;
constexpr double PI = 3.14159265358979323846;

// 3D Vector class with all necessary operations
class Vec3 {
public:
    double x, y, z;
    
    // Constructors
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    
    // Vector operations
    Vec3 operator+(const Vec3& v) const {
        return Vec3(x + v.x, y + v.y, z + v.z);
    }
    
    Vec3 operator-(const Vec3& v) const {
        return Vec3(x - v.x, y - v.y, z - v.z);
    }
    
    Vec3 operator*(double scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }
    
    Vec3 operator/(double scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }
    
    Vec3& operator+=(const Vec3& v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }
    
    Vec3& operator-=(const Vec3& v) {
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }
    
    Vec3& operator*=(double scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }
    
    // Dot product
    double dot(const Vec3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    
    // Cross product
    Vec3 cross(const Vec3& v) const {
        return Vec3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }
    
    // Length and normalization
    double length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    double lengthSquared() const {
        return x * x + y * y + z * z;
    }
    
    Vec3 normalized() const {
        double len = length();
        if (len < EPSILON) {
            return Vec3(0, 0, 0);
        }
        return *this / len;
    }
    
    void normalize() {
        double len = length();
        if (len > EPSILON) {
            x /= len; y /= len; z /= len;
        }
    }
    
    // Utility
    bool isZero() const {
        return std::abs(x) < EPSILON && std::abs(y) < EPSILON && std::abs(z) < EPSILON;
    }
};

// Scalar multiplication (scalar * vector)
inline Vec3 operator*(double scalar, const Vec3& v) {
    return v * scalar;
}

// 2D vector for parameter space
class Vec2 {
public:
    double u, v;
    
    Vec2() : u(0), v(0) {}
    Vec2(double u, double v) : u(u), v(v) {}
    
    Vec2 operator+(const Vec2& other) const {
        return Vec2(u + other.u, v + other.v);
    }
    
    Vec2 operator-(const Vec2& other) const {
        return Vec2(u - other.u, v - other.v);
    }
    
    Vec2 operator*(double scalar) const {
        return Vec2(u * scalar, v * scalar);
    }
    
    Vec2 operator/(double scalar) const {
        return Vec2(u / scalar, v / scalar);
    }
    
    double length() const {
        return std::sqrt(u * u + v * v);
    }
};

// First fundamental form (metric tensor)
struct FirstFundamentalForm {
    double E; // <r_u, r_u>
    double F; // <r_u, r_v>
    double G; // <r_v, r_v>
    
    double determinant() const {
        return E * G - F * F;
    }
    
    // Inverse metric tensor components
    double E_inv() const {
        double det = determinant();
        return (det > EPSILON) ? (G / det) : 0.0;
    }
    
    double F_inv() const {
        double det = determinant();
        return (det > EPSILON) ? (-F / det) : 0.0;
    }
    
    double G_inv() const {
        double det = determinant();
        return (det > EPSILON) ? (E / det) : 0.0;
    }
};

// Second fundamental form (curvature tensor)
struct SecondFundamentalForm {
    double L; // <r_uu, N>
    double M; // <r_uv, N>
    double N; // <r_vv, N>
};

// Christoffel symbols of the second kind
struct ChristoffelSymbols {
    // Γ^1_11, Γ^1_12, Γ^1_21, Γ^1_22
    double gamma1_11, gamma1_12, gamma1_21, gamma1_22;
    // Γ^2_11, Γ^2_12, Γ^2_21, Γ^2_22
    double gamma2_11, gamma2_12, gamma2_21, gamma2_22;
    
    ChristoffelSymbols() : 
        gamma1_11(0), gamma1_12(0), gamma1_21(0), gamma1_22(0),
        gamma2_11(0), gamma2_12(0), gamma2_21(0), gamma2_22(0) {}
};

// Curvature information
struct CurvatureInfo {
    double gaussian;  // K = (LN - M²)/(EG - F²)
    double mean;      // H = (EN - 2FM + GL)/(2(EG - F²))
    Vec3 normal;      // Unit normal vector
    
    CurvatureInfo() : gaussian(0), mean(0), normal(0, 0, 1) {}
};

// Geodesic state for integration
struct GeodesicState {
    Vec2 position;    // (u, v) parameter space position
    Vec2 velocity;    // (du/dt, dv/dt) parameter space velocity
    Vec3 point3D;     // Corresponding 3D point
    
    GeodesicState() : position(0, 0), velocity(0, 0), point3D(0, 0, 0) {}
    GeodesicState(const Vec2& pos, const Vec2& vel) 
        : position(pos), velocity(vel), point3D(0, 0, 0) {}
};

#endif // GEOMETRY_HPP