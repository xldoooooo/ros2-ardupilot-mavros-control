// Bounded voxel evidence: unseen is unknown; only measured rays retire occupied cells.
#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace avoidance_bridge {
using Vec = std::array<double, 3>;
using Key = std::array<int, 3>;
struct Hash {
  size_t operator()(const Key &k) const {
    return size_t(k[0]) * 73856093u ^ size_t(k[1]) * 19349663u ^ size_t(k[2]) * 83492791u;
  }
};
struct Cell { bool occupied; Vec point; };

class ObservedMap {
public:
  ObservedMap(double voxel, double radius, size_t cap): voxel_(voxel), radius_(radius), cap_(cap) {
    if(!std::isfinite(voxel)||voxel<.02||voxel>1||!std::isfinite(radius)||radius<=0||radius>100||cap<1||cap>1000000)
      throw std::invalid_argument("invalid observed map bounds");
  }
  void clear() { cells_.clear(); anchored_=false; saturated_=false; }
  Key key(const Vec &p) const {
    return {int(std::floor(p[0]/voxel_)),int(std::floor(p[1]/voxel_)),int(std::floor(p[2]/voxel_))};
  }
  bool observed(const Vec &p) const {
    for(double v:p) if(!std::isfinite(v)||std::abs(v)>1e6) return false;
    const auto it=cells_.find(key(p));
    return !saturated_ && it!=cells_.end() && !it->second.occupied;
  }
  bool saturated() const { return saturated_; }
  double voxel() const { return voxel_; }
  std::vector<Vec> occupied() const {
    std::vector<Vec> points;
    for (const auto &item:cells_) if(item.second.occupied) points.push_back(item.second.point);
    return points;
  }
  // Rays refer to one registered scan, not an accumulated SLAM cloud. Endpoints win over all rays.
  void update(const Vec &origin, const std::vector<Vec> &points) {
    for(double v:origin) if(!std::isfinite(v)||std::abs(v)>1e6) {saturated_=true;return;}
    if(!anchored_) { anchor_=origin; anchored_=true; }
    // Bound ray work independently of raw density. Every endpoint still becomes occupied below.
    const size_t stride=std::max(size_t(1),(points.size()+9999)/10000);
    for(size_t i=0;i<points.size();i+=stride) ray(origin,points[i]);
    for(const Vec &p:points) if(inside(p)) put(key(p),{true,p});
  }
private:
  bool inside(const Vec &p) const {
    for(double v:p) if(!std::isfinite(v)||std::abs(v)>1e6) return false;
    double d=0; for(size_t i=0;i<3;++i) d+=(p[i]-anchor_[i])*(p[i]-anchor_[i]);
    return d<=radius_*radius_;
  }
  void put(const Key &k, const Cell &cell) {
    auto it=cells_.find(k);
    if(it!=cells_.end()) { it->second=cell; return; }
    if(cells_.size()>=cap_) { saturated_=true; return; }
    cells_.emplace(k,cell);
  }
  // Half-voxel sampling is conservative for free evidence: missed crossed cells remain unknown.
  // Stop sqrt(3)*voxel before the hit so endpoint quantization cannot erase the foreground hit.
  void ray(const Vec &origin,const Vec &hit) {
    Vec delta; double length=0;
    for(size_t i=0;i<3;++i) { delta[i]=hit[i]-origin[i]; length+=delta[i]*delta[i]; }
    length=std::sqrt(length);
    if(length<=std::sqrt(3.0)*voxel_) return;
    const double end=std::min(length-std::sqrt(3.0)*voxel_,2*radius_);
    for(double d=0;d<end;d+=voxel_/2) {
      Vec p; for(size_t i=0;i<3;++i) p[i]=origin[i]+delta[i]*d/length;
      if(inside(p)) put(key(p),{false,p});
    }
  }
  double voxel_,radius_; size_t cap_;
  bool anchored_=false,saturated_=false;
  Vec anchor_{};
  std::unordered_map<Key,Cell,Hash> cells_;
};
}  // namespace avoidance_bridge
