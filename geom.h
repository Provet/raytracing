#pragma once

#include <cmath>
#include <iostream>
#include <memory>

using namespace std;

const double INF = 1e9;
const double EPS = 1e-9;

bool LessOrEqual(double lhs, double rhs) { return lhs < rhs + EPS; }

bool InRange(double l, double r, double x) {
  return LessOrEqual(l, x) && LessOrEqual(x, r);
}

double DegreesToRadians(double degrees) { return degrees * M_PI / 180.0; }

class Point {
public:
  Point() {}

  Point(double x, double y, double z) : x(x), y(y), z(z) {}

  double x;
  double y;
  double z;
};

class Vector {
public:
  Vector() {}

  Vector(Point a, Point b) {
    x = b.x - a.x;
    y = b.y - a.y;
    z = b.z - a.z;
  }

  Vector(double x, double y, double z) : x(x), y(y), z(z) {}

  Point Shift(const Point &p) const { return Point(p.x + x, p.y + y, p.z + z); }

  double Len() const { return sqrt(DotProduct(*this, *this)); }

  Vector Norm() const {
    double len = Len();
    return Vector(x / len, y / len, z / len);
  }

  Vector Mul(double k) const { return Vector(x * k, y * k, z * k); }

  Vector Add(const Vector &rhs) const {
    return Vector(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  Vector Sub(const Vector &rhs) const { return Add(rhs.Mul(-1)); }

  Vector Reflect(const Vector &n) const {
    Vector L = this->Mul(-1).Norm();
    Vector Ln = n.Mul(DotProduct(L, n));
    Vector Lp = L.Sub(Ln);
    Vector R = Ln.Add(Lp.Mul(-1));
    return R;
  }

  static double DotProduct(const Vector &lhs, const Vector &rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
  }

  static Vector CrossProduct(const Vector &lhs, const Vector &rhs) {
    return Vector(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z,
                  lhs.x * rhs.y - lhs.y * rhs.x);
  }

  double x;
  double y;
  double z;
};

class Color {
public:
  Color() {}

  Color(double r, double g, double b) : r(r), g(g), b(b) {}

  Color Mul(double k) const { return Color(r * k, g * k, b * k); }

  Color Add(Color c) const {
    return Color(min(1.0, r + c.r), min(1.0, g + c.g), min(1.0, b + c.b));
  }

  double r;
  double g;
  double b;
};

class Material {
public:
  Material() {}

  Material(Color color)
      : color(color), specular(500), diffuseAlbedo(0.6), specularAlbedo(0.3),
        reflactive(0.05) {}

  Material(Color color, double specular, double diffuseAlbedo,
           double specularAlbedo, double reflactive)
      : color(color), specular(specular), diffuseAlbedo(diffuseAlbedo),
        specularAlbedo(specularAlbedo), reflactive(reflactive) {}

  Color color;
  double diffuseAlbedo;
  double specularAlbedo;
  double specular;
  double reflactive;
};

const double G = 0.5;

class Sphere {
public:
  Sphere() {}

  Sphere(Point center, double radius, Material material)
      : center(center), radius(radius), material(material) {
    this->material.color =
        Color(pow(material.color.r, G), pow(material.color.g, G),
              pow(material.color.b, G));
  }

  // returns distance to intersection point from o
  shared_ptr<pair<double, double>> IntersectRay(const Point &o,
                                                const Vector &dir) const {
    Vector co(center, o);
    double k1 = Vector::DotProduct(dir, dir);
    double k2 = 2 * Vector::DotProduct(co, dir);
    double k3 = Vector::DotProduct(co, co) - radius * radius;
    double discriminant = k2 * k2 - 4 * k1 * k3;
    if (LessOrEqual(discriminant, 0)) {
      return nullptr;
    }
    double t1 = (-k2 + sqrt(discriminant)) / (2 * k1);
    double t2 = (-k2 - sqrt(discriminant)) / (2 * k1);
    return make_shared<pair<double, double>>(t1, t2);
  }

  Point center;
  double radius;
  Material material;
};

class Triangle {
public:
  Triangle() {}

  Triangle(Point a, Point b, Point c, Material material)
      : a(a), b(b), c(c), material(material) {
    this->material.color =
        Color(pow(material.color.r, G), pow(material.color.g, G),
              pow(material.color.b, G));
  }

  Vector Normal() const {
    Vector ab(a, b);
    Vector ac(a, c);
    return Vector::CrossProduct(ab, ac).Norm();
  }

  shared_ptr<double> IntersectRay(const Point &o, const Vector &dir) const {
    // double A = a.y * (b.z - c.z) + b.y * (c.z - a.z) + c.y * (a.z - b.z);
    // double B = a.z * (b.x - c.x) + b.z * (c.x - a.x) + c.z * (a.x - b.x);
    // double C = a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y);
    // double D = -a.x * (b.y * c.z - c.y * b.z) - b.x * (c.y * a.z - a.y * c.z)
    // -
    //            c.x * (a.y * b.z - b.y * a.z);

    Vector n = Normal();
    Vector v = Vector(o, a);
    double d = Vector::DotProduct(n, v);
    double e = Vector::DotProduct(n, dir);
    if (fabs(e) < EPS) {
      return nullptr;
    }
    double t = d / e;
    Point p = dir.Norm().Mul(t).Shift(o);
    Vector pa(p, a);
    Vector pb(p, b);
    Vector pc(p, c);
    Vector n1 = Vector::CrossProduct(pa, pb);
    Vector n2 = Vector::CrossProduct(pb, pc);
    Vector n3 = Vector::CrossProduct(pc, pa);
    if (Vector::DotProduct(n, n1) >= 0 && Vector::DotProduct(n, n2) >= 0 &&
        Vector::DotProduct(n, n3) >= 0) {
      return make_shared<double>(t);
    }
    return nullptr;
  }

  Point a;
  Point b;
  Point c;
  Material material;
};
