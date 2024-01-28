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
    {"background", Material(BACKGROUND_COLOR)},
    {"mirror", Material(Color(1.0, 1.0, 1.0), 1400 /* specular */,
                        0.0 /* diffuse albedo */, 10.0 /* specular albedo */,
                        0.9 /* reflactive */)},
    {"default", Material(Color(0.4, 0.4, 0.3))},
    {"blue_rubber", Material(Color(0.2, 0.2, 0.8), 9, 0.9, 0.1, 0.0)},
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

pair<Material, shared_ptr<pair<Point, Vector>>>
CastRay(const Point &camera, const Vector &ray, const vector<Sphere> &spheres,
        const vector<Light> &lights, double tmin, double tmax) {
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
    return {MATERIALS.at("background"), nullptr};
  }

  Point hit = ray.Norm().Mul(closestT).Shift(camera);
  Vector norm = Vector(closestSphere->center, hit).Norm();
  return {closestSphere->material, make_shared<pair<Point, Vector>>(hit, norm)};
}

Color ComputeLight(const Point &camera, const Vector &ray,
                   const vector<Sphere> &spheres, const vector<Light> &lights,
                   const Point &hit, const Vector &norm, const Material &mat) {
  double diffuseLight = 0;
  double specularLight = 0;

  for (const Light &light : lights) {
    Vector rayToLight = Vector(hit, light.position).Norm();
    double diffuseLightDelta =
        light.intensity * max(0.0, Vector::DotProduct(rayToLight, norm));
    Vector rayFromLight = Vector(light.position, hit).Norm();
    Vector R = rayFromLight.Reflect(norm);
    double specularLightDelta = 0;

    if (mat.specular != -1) {
      specularLightDelta =
          pow(max(0.0, Vector::DotProduct(ray.Mul(-1).Norm(), R)),
              mat.specular) *
          light.intensity;
    }

    if (Vector::DotProduct(rayToLight, norm) > 0) {
      auto sh = CastRay(hit, rayToLight, spheres, lights, EPS, INF);
      if (sh.second != nullptr) {
        diffuseLightDelta *= 0.5;
        specularLightDelta = 0;
      }
    }

    diffuseLight += diffuseLightDelta;
    specularLight += specularLightDelta;
  }

  return mat.color.Mul(diffuseLight * mat.diffuseAlbedo)
      .Add(Color(1, 1, 1).Mul(specularLight * mat.specularAlbedo));
}

pair<Color, shared_ptr<pair<Point, Vector>>>
RayCasting(const Point &camera, const Vector &ray,
           const vector<Sphere> &spheres, const vector<Light> &lights,
           double tmin, double tmax, int depth = 0) {
  auto res = CastRay(camera, ray, spheres, lights, tmin, tmax);
  if (res.second == nullptr) {
    return {res.first.color, nullptr};
  }

  auto mat = res.first;
  auto hit = res.second->first;
  auto norm = res.second->second;

  auto localColor = ComputeLight(camera, ray, spheres, lights, hit, norm, mat);
  double r = mat.reflactive;
  if (depth > 4 || r == 0) {
    return {localColor, res.second};
  }

  auto reflectedRay = ray.Reflect(norm);
  /*auto reflectedHit = Vector::DotProduct(reflectedRay, norm) < 0
                          ? norm.Mul(0.01).Shift(hit)
                          : norm.Mul(-0.01).Shift(hit);*/
  auto reflectedColor =
      RayCasting(hit, reflectedRay, spheres, lights, EPS, INF, depth + 1).first;

  return {localColor.Add(reflectedColor.Mul(r)), res.second};
}

Color RayTracing(const Point &camera, const Vector &ray,
                 const vector<Sphere> &spheres, const vector<Light> &lights,
                 double tmin, double tmax) {
  auto res = RayCasting(camera, ray, spheres, lights, tmin, tmax);
  return res.first;
}

void render(const Point &camera, const vector<Sphere> &spheres,
            const vector<Light> &lights) {
  const int W = 2048;
  const int H = 2048;
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
  vector<Sphere> spheres = {
      Sphere(Point(0, 0, 25), 3, MATERIALS.at("default")),
      Sphere(Point(-7, 2, 20), 3, MATERIALS.at("mirror")),
      Sphere(Point(3, -5, 25), 4, MATERIALS.at("blue_rubber")),
      Sphere(Point(-3, -4, 25), 1, MATERIALS.at("green")),
      Sphere(Point(1, 4, 20), 2, MATERIALS.at("blue_rubber")),
      Sphere(Point(-3, 5, 25), 2, MATERIALS.at("red"))};
  vector<Light> lights = {Light(Point(-5, 10, 0), 1.3),
                          Light(Point(-20, -30, 50), 1.5),
                          Light(Point(40, 20, 10), 1.1)};
  Point camera(0, 0, 0);
  render(camera, spheres, lights);
}
