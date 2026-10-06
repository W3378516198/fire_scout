#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fire_scout {
constexpr double pi = 3.14159265358979323846;
struct Vec3 {
  double x{0}, y{0}, z{0};
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
inline double distance(Vec3 a, Vec3 b) { return norm(a - b); }
inline bool finite(Vec3 a) {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
inline double wrap(double a) { return std::remainder(a, 2 * pi); }
inline Vec3 limitXY(Vec3 a, double m) {
  const double n = std::hypot(a.x, a.y);
  if (n > m) {
    a.x *= m / n;
    a.y *= m / n;
  }
  return a;
}
inline Vec3 limitNorm(Vec3 a, double m) {
  const double n = norm(a);
  return n > m ? a * (m / n) : a;
}
inline double pathLength(const std::vector<Vec3> &p) {
  double d = 0;
  for (size_t i = 1; i < p.size(); ++i)
    d += distance(p[i - 1], p[i]);
  return d;
}
struct Projection {
  Vec3 point{};
  double s{0}, error{std::numeric_limits<double>::infinity()};
  size_t segment{0};
};
inline std::vector<double> arcLengths(const std::vector<Vec3> &p) {
  std::vector<double> s(p.size(), 0);
  for (size_t i = 1; i < p.size(); ++i)
    s[i] = s[i - 1] + distance(p[i], p[i - 1]);
  return s;
}
inline Projection project(const std::vector<Vec3> &p, const std::vector<double> &arc, Vec3 q,
                          double lo = 0, double hi = 1e100) {
  Projection best;
  if (p.size() == 1)
    return {p[0], 0, distance(q, p[0]), 0};
  for (size_t i = 1; i < p.size(); ++i) {
    double len = arc[i] - arc[i - 1];
    if (len < 1e-9 || arc[i] < lo || arc[i - 1] > hi)
      continue;
    double a = std::clamp((lo - arc[i - 1]) / len, 0.0, 1.0),
           b = std::clamp((hi - arc[i - 1]) / len, 0.0, 1.0);
    Vec3 d = p[i] - p[i - 1];
    double u = std::clamp(dot(q - p[i - 1], d) / (len * len), a, b);
    Vec3 point = p[i - 1] + d * u;
    double e = distance(point, q);
    if (e < best.error - 1e-8)
      best = {point, arc[i - 1] + u * len, e, i - 1};
  }
  return best;
}
inline Vec3 atArc(const std::vector<Vec3> &p, const std::vector<double> &arc, double s) {
  if (p.empty())
    return {};
  if (s <= 0)
    return p.front();
  for (size_t i = 1; i < p.size(); ++i)
    if (s <= arc[i])
      return p[i - 1] +
             (p[i] - p[i - 1]) * ((s - arc[i - 1]) / std::max(1e-9, arc[i] - arc[i - 1]));
  return p.back();
}
inline std::vector<Vec3> densify(const std::vector<Vec3> &p, double step) {
  if (p.empty())
    return {};
  std::vector<Vec3> out{p.front()};
  for (size_t i = 1; i < p.size(); ++i) {
    int n = std::max(1, int(std::ceil(distance(p[i], p[i - 1]) / step)));
    for (int j = 1; j <= n; ++j)
      out.push_back(p[i - 1] + (p[i] - p[i - 1]) * (double(j) / n));
  }
  return out;
}
} // namespace fire_scout
