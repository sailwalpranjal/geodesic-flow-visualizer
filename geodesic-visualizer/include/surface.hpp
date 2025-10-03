#ifndef SURFACE_HPP
#define SURFACE_HPP

#include "geometry.hpp"
#include <string>
#include <memory>

// Abstract base class for parametric surfaces
class Surface {
public:
    virtual ~Surface() = default;
    
    // Surface evaluation at parameter (u, v)
    virtual Vec3 eval(double u, double v) const = 0;
    
    // First-order partial derivatives
    virtual Vec3 eval_du(double u, double v) const = 0;  // ∂r/∂u
    virtual Vec3 eval_dv(double u, double v) const = 0;  // ∂r/∂v
    
    // Second-order partial derivatives
    virtual Vec3 eval_duu(double u, double v) const = 0; // ∂²r/∂u²
    virtual Vec3 eval_duv(double u, double v) const = 0; // ∂²r/∂u∂v
    virtual Vec3 eval_dvv(double u, double v) const = 0; // ∂²r/∂v²
    
    // Parameter bounds
    virtual double uMin() const = 0;
    virtual double uMax() const = 0;
    virtual double vMin() const = 0;
    virtual double vMax() const = 0;
    
    // Surface name
    virtual std::string name() const = 0;
    
    // Compute unit normal vector at (u, v)
    Vec3 normal(double u, double v) const {
        Vec3 r_u = eval_du(u, v);
        Vec3 r_v = eval_dv(u, v);
        Vec3 n = r_u.cross(r_v);
        return n.normalized();
    }
    
    // Check if parameter values are within valid range
    bool isValidParameter(double u, double v) const {
        return u >= uMin() && u <= uMax() && v >= vMin() && v <= vMax();
    }
    
    // Clamp parameters to valid range
    void clampParameters(double& u, double& v) const {
        u = std::max(uMin(), std::min(uMax(), u));
        v = std::max(vMin(), std::min(vMax(), v));
    }
};

// Torus: (R + r*cos(v))*cos(u), (R + r*cos(v))*sin(u), r*sin(v)
class Torus : public Surface {
private:
    double R; // Major radius
    double r; // Minor radius
    
public:
    Torus(double majorRadius = 2.0, double minorRadius = 0.5) 
        : R(majorRadius), r(minorRadius) {}
    
    Vec3 eval(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        double sinv = std::sin(v);
        
        double x = (R + r * cosv) * cosu;
        double y = (R + r * cosv) * sinu;
        double z = r * sinv;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_du(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        
        double x = -(R + r * cosv) * sinu;
        double y = (R + r * cosv) * cosu;
        double z = 0;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_dv(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        double sinv = std::sin(v);
        
        double x = -r * sinv * cosu;
        double y = -r * sinv * sinu;
        double z = r * cosv;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_duu(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        
        double x = -(R + r * cosv) * cosu;
        double y = -(R + r * cosv) * sinu;
        double z = 0;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_duv(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double sinv = std::sin(v);
        
        double x = r * sinv * sinu;
        double y = -r * sinv * cosu;
        double z = 0;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_dvv(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        double sinv = std::sin(v);
        
        double x = -r * cosv * cosu;
        double y = -r * cosv * sinu;
        double z = -r * sinv;
        
        return Vec3(x, y, z);
    }
    
    double uMin() const override { return 0.0; }
    double uMax() const override { return 2.0 * PI; }
    double vMin() const override { return 0.0; }
    double vMax() const override { return 2.0 * PI; }
    
    std::string name() const override { return "Torus"; }
};

// Sphere: r*sin(v)*cos(u), r*sin(v)*sin(u), r*cos(v)
class Sphere : public Surface {
private:
    double r; // Radius
    
public:
    Sphere(double radius = 2.0) : r(radius) {}
    
    Vec3 eval(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        double sinv = std::sin(v);
        
        double x = r * sinv * cosu;
        double y = r * sinv * sinu;
        double z = r * cosv;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_du(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double sinv = std::sin(v);
        
        double x = -r * sinv * sinu;
        double y = r * sinv * cosu;
        double z = 0;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_dv(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        double sinv = std::sin(v);
        
        double x = r * cosv * cosu;
        double y = r * cosv * sinu;
        double z = -r * sinv;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_duu(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double sinv = std::sin(v);
        
        double x = -r * sinv * cosu;
        double y = -r * sinv * sinu;
        double z = 0;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_duv(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        
        double x = -r * cosv * sinu;
        double y = r * cosv * cosu;
        double z = 0;
        
        return Vec3(x, y, z);
    }
    
    Vec3 eval_dvv(double u, double v) const override {
        double cosu = std::cos(u);
        double sinu = std::sin(u);
        double cosv = std::cos(v);
        double sinv = std::sin(v);
        
        double x = -r * sinv * cosu;
        double y = -r * sinv * sinu;
        double z = -r * cosv;
        
        return Vec3(x, y, z);
    }
    
    double uMin() const override { return 0.0; }
    double uMax() const override { return 2.0 * PI; }
    double vMin() const override { return 0.1; }  // Avoid poles
    double vMax() const override { return PI - 0.1; }
    
    std::string name() const override { return "Sphere"; }
};

// Hyperbolic Paraboloid (Saddle): u, v, u² - v²
class HyperbolicParaboloid : public Surface {
private:
    double scale;
    
public:
    HyperbolicParaboloid(double s = 0.2) : scale(s) {}
    
    Vec3 eval(double u, double v) const override {
        return Vec3(u, v, scale * (u * u - v * v));
    }
    
    Vec3 eval_du(double u, double v) const override {
        return Vec3(1.0, 0.0, 2.0 * scale * u);
    }
    
    Vec3 eval_dv(double u, double v) const override {
        return Vec3(0.0, 1.0, -2.0 * scale * v);
    }
    
    Vec3 eval_duu(double u, double v) const override {
        return Vec3(0.0, 0.0, 2.0 * scale);
    }
    
    Vec3 eval_duv(double u, double v) const override {
        return Vec3(0.0, 0.0, 0.0);
    }
    
    Vec3 eval_dvv(double u, double v) const override {
        return Vec3(0.0, 0.0, -2.0 * scale);
    }
    
    double uMin() const override { return -3.0; }
    double uMax() const override { return 3.0; }
    double vMin() const override { return -3.0; }
    double vMax() const override { return 3.0; }
    
    std::string name() const override { return "Hyperbolic Paraboloid"; }
};

// Monkey Saddle: u, v, u³ - 3uv²
class MonkeySaddle : public Surface {
private:
    double scale;
    
public:
    MonkeySaddle(double s = 0.15) : scale(s) {}
    
    Vec3 eval(double u, double v) const override {
        return Vec3(u, v, scale * (u * u * u - 3.0 * u * v * v));
    }
    
    Vec3 eval_du(double u, double v) const override {
        return Vec3(1.0, 0.0, scale * (3.0 * u * u - 3.0 * v * v));
    }
    
    Vec3 eval_dv(double u, double v) const override {
        return Vec3(0.0, 1.0, scale * (-6.0 * u * v));
    }
    
    Vec3 eval_duu(double u, double v) const override {
        return Vec3(0.0, 0.0, scale * 6.0 * u);
    }
    
    Vec3 eval_duv(double u, double v) const override {
        return Vec3(0.0, 0.0, scale * (-6.0 * v));
    }
    
    Vec3 eval_dvv(double u, double v) const override {
        return Vec3(0.0, 0.0, scale * (-6.0 * u));
    }
    
    double uMin() const override { return -2.0; }
    double uMax() const override { return 2.0; }
    double vMin() const override { return -2.0; }
    double vMax() const override { return 2.0; }
    
    std::string name() const override { return "Monkey Saddle"; }
};

// Factory function for creating surfaces
inline std::unique_ptr<Surface> createSurface(const std::string& type) {
    if (type == "torus") {
        return std::make_unique<Torus>(2.0, 0.5);
    } else if (type == "sphere") {
        return std::make_unique<Sphere>(2.0);
    } else if (type == "hyperbolic_paraboloid") {
        return std::make_unique<HyperbolicParaboloid>(0.2);
    } else if (type == "monkey_saddle") {
        return std::make_unique<MonkeySaddle>(0.15);
    }
    return std::make_unique<Torus>(2.0, 0.5); // Default
}

#endif // SURFACE_HPP