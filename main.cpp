#include "geom.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

using namespace std;

const Color BACKGROUND_COLOR = Color(0.2, 0.7, 0.8);
const map<string, Material> MATERIALS = {
    {"default", Material(Color(0.4, 0.4, 0.3))},
    {"blue", Material(Color(0.2, 0.2, 0.8))},
    {"green", Material(Color(0.2, 0.8, 0.2))},
    {"red", Material(Color(0.8, 0.2, 0.2))},
};

class Light {
public:
  Light() {}

  Light(Point position, double intensity)
      : position(position), intensity(intensity) {}

  Point position;
  double intensity;
};

pair<Color, shared_ptr<pair<Point, Vector>>>
RayCasting(const Point &camera, const Vector &ray,
           const vector<Sphere> &spheres, const vector<Light> &lights,
           double tmin, double tmax, bool onlyIntersection = false) {
  double closestT = INF;
  shared_ptr<Sphere> closestSphere = nullptr;

  for (const Sphere &sphere : spheres) {
    auto ts = sphere.IntersectRay(camera, ray);
    if (ts == nullptr) {
      continue;
    }
    if (InRange(tmin, tmax, ts->first) && LessOrEqual(ts->first, closestT)) {
      closestT = ts->first;
      closestSphere = make_shared<Sphere>(sphere);
    }
    if (InRange(tmin, tmax, ts->second) && LessOrEqual(ts->second, closestT)) {
      closestT = ts->second;
      closestSphere = make_shared<Sphere>(sphere);
    }
  }

  if (closestSphere == nullptr) {
    return {BACKGROUND_COLOR, nullptr};
  }

  Point hit = ray.Norm().Mul(closestT).Shift(camera);
  Vector norm = Vector(closestSphere->center, hit).Norm();
  if (onlyIntersection) {
    return {Color(0, 0, 0), make_shared<pair<Point, Vector>>(hit, norm)};
  }

  double diffuseLight = 0;
  double specularLight = 0;

  for (const Light &light : lights) {
    Vector rayToLight = Vector(hit, light.position).Norm();
    double diffuseLightDelta =
        light.intensity * max(0.0, Vector::DotProduct(rayToLight, norm));
    Vector rayFromLight = Vector(light.position, hit).Norm();
    Vector R = rayFromLight.Reflect(norm);
    double specularLightDelta =
        max(0.0, Vector::DotProduct(ray.Mul(-1).Norm(), R));

    if (Vector::DotProduct(rayToLight, norm) > 0) {
      auto sh = RayCasting(hit, rayToLight, spheres, lights, EPS, INF, true);
      if (sh.second != nullptr) {
        diffuseLightDelta *= 0.5;
        specularLightDelta = 0;
      }
    }

    diffuseLight += diffuseLightDelta;
    specularLight += specularLightDelta;
  }

  return {closestSphere->material.color.Mul(diffuseLight)
              .Add(Color(1, 1, 1).Mul(pow(specularLight, 65))),
          make_shared<pair<Point, Vector>>(hit, norm)};
}

Color RayTracing(const Point &camera, const Vector &ray,
                 const vector<Sphere> &spheres, const vector<Light> &lights,
                 double tmin, double tmax) {
  auto res = RayCasting(camera, ray, spheres, lights, tmin, tmax, false);
  return res.first;
}

void render(const Point &camera, const vector<Sphere> &spheres,
            const vector<Light> &lights) {
  const int W = 1024;
  const int H = 1024;
  const double Fw = 1;
  const double Fh = 1;

  vector<Color> frame(W * H);

  for (int i = 0; i < H; ++i) {
    for (int j = 0; j < W; ++j) {
      int Cy = i - H / 2;
      int Cx = j - W / 2;
      double x = Cx * Fw / W;
      double y = -Cy * Fh / H; // y is inverted for some reason. think about it
      double z = 1;
      Vector ray(camera, Point(x, y, z));
      frame[i * W + j] = RayTracing(camera, ray, spheres, lights, 1.0, INF);
    }
  }

  ofstream out;
  out.open("img.ppm");
  out << "P3\n" << W << " " << H << "\n255\n";

  for (int i = 0; i < W * H; ++i) {
    out << int(255 * frame[i].r) << " " << int(255 * frame[i].g) << " "
        << int(255 * frame[i].b) << "\n";
  }
}

int main() {
  vector<Sphere> spheres = {Sphere(Point(0, 0, 25), 3, MATERIALS.at("default")),
                            Sphere(Point(3, -5, 25), 4, MATERIALS.at("blue")),
                            Sphere(Point(-3, -4, 25), 1, MATERIALS.at("green")),
                            Sphere(Point(1, 4, 20), 2, MATERIALS.at("blue")),
                            Sphere(Point(-3, 5, 25), 2, MATERIALS.at("red"))};
  vector<Light> lights = {Light(Point(-5, 10, 0), 1.7),
                          Light(Point(-20, -30, 50), 3.0)};
  Point camera(0, 0, 0);
  render(camera, spheres, lights);
}
