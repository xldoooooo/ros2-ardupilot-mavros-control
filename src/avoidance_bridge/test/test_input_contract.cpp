// Malformed PointCloud2 layouts and source clocks must never be interpreted as XYZ scans.
#include "avoidance_bridge/input_contract.hpp"
#include <cassert>
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
int main() {
  PointCloud2 cloud;cloud.width=1;cloud.height=1;cloud.point_step=12;cloud.row_step=12;cloud.data.resize(12);
  for(size_t i=0;i<3;++i) {PointField f;f.name=std::array<const char*,3>{"x","y","z"}[i];f.offset=i*4;f.datatype=7;f.count=1;cloud.fields.push_back(f);}
  std::array<uint32_t,3> offsets;
  assert(avoidance_bridge::xyz_schema(cloud,offsets));
  auto bad=cloud;bad.fields[2].name="x";assert(!avoidance_bridge::xyz_schema(bad,offsets));
  bad=cloud;bad.fields.push_back(bad.fields[0]);assert(!avoidance_bridge::xyz_schema(bad,offsets));
  bad=cloud;bad.fields[2].offset=1;assert(!avoidance_bridge::xyz_schema(bad,offsets));
  bad=cloud;bad.fields[2].offset=UINT32_MAX;assert(!avoidance_bridge::xyz_schema(bad,offsets));
  bad=cloud;bad.fields[2].count=2;assert(!avoidance_bridge::xyz_schema(bad,offsets));
  bad=cloud;bad.fields[2].datatype=8;assert(!avoidance_bridge::xyz_schema(bad,offsets));
  bad=cloud;bad.data.push_back(0);assert(!avoidance_bridge::xyz_schema(bad,offsets));
  bad=cloud;bad.data.pop_back();assert(!avoidance_bridge::xyz_schema(bad,offsets));
  builtin_interfaces::msg::Time t;assert(avoidance_bridge::valid_stamp(t));
  t.sec=-1;assert(!avoidance_bridge::valid_stamp(t));
  t.sec=0;t.nanosec=1000000000;assert(!avoidance_bridge::valid_stamp(t));
  t.nanosec=999999999;assert(avoidance_bridge::valid_stamp(t));
}
