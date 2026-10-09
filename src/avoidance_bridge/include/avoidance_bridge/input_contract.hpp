// Validate source-clock fields and exact XYZ cloud layouts before interpreting bytes or throttling.
#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <cstdint>
#include <string>
namespace avoidance_bridge {
inline bool valid_stamp(const builtin_interfaces::msg::Time &t) {return t.sec>=0 && t.nanosec<1000000000u;}
inline int64_t source_ns(const builtin_interfaces::msg::Time &t) {return int64_t(t.sec)*1000000000ll+t.nanosec;}
inline bool xyz_schema(const sensor_msgs::msg::PointCloud2 &c,std::array<uint32_t,3> &offsets) {
  if(c.is_bigendian||c.point_step<12||c.row_step<uint64_t(c.point_step)*c.width||
     c.data.size()!=uint64_t(c.row_step)*c.height) return false;
  std::array<bool,3> found{};
  constexpr std::array<const char *,3> names{"x","y","z"};
  for(const auto &field:c.fields) for(size_t i=0;i<3;++i) if(field.name==names[i]) {
    if(found[i]||field.datatype!=sensor_msgs::msg::PointField::FLOAT32||field.count!=1||
       uint64_t(field.offset)+4>c.point_step) return false;
    offsets[i]=field.offset;found[i]=true;
  }
  if(!found[0]||!found[1]||!found[2]) return false;
  for(size_t i=0;i<3;++i) for(size_t j=i+1;j<3;++j)
    if(uint64_t(offsets[i])<uint64_t(offsets[j])+4&&uint64_t(offsets[j])<uint64_t(offsets[i])+4) return false;
  return true;
}
}  // namespace avoidance_bridge
