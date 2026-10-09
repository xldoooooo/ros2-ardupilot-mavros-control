// Regression of persistent occupancy, measured free retirement and bounded unknown evidence.
#include "avoidance_bridge/observed_map.hpp"
#include <cassert>
#include <limits>
using namespace avoidance_bridge;
int main() {
  ObservedMap map(.1,10,10000);
  map.update({0,0,0},{{2,0,0}});
  assert(map.occupied().size()==1 && !map.observed({2,0,0}));
  map.update({0,0,0},{{0,2,0}}); // A different ray cannot erase an occluded obstacle.
  assert(map.occupied().size()==2);
  map.update({0,0,0},{{4,0,0}}); // A farther hit establishes free space through the former obstacle.
  assert(map.observed({2,0,0}) && map.occupied().size()==2);
  assert(!map.observed({1,1,0}));
  map.update({0,0,0},{});
  assert(map.occupied().size()==2);
  ObservedMap tiny(.1,10,2);
  tiny.update({0,0,0},{{4,0,0}});
  assert(tiny.saturated() && !tiny.observed({0,0,0}));
  map.clear(); assert(map.occupied().empty() && !map.observed({0,0,0}));
  for(const auto &params:std::vector<Vec>{{0,10,100},{.1,-1,100},{std::numeric_limits<double>::quiet_NaN(),10,100},
                                      {.1,std::numeric_limits<double>::infinity(),100},{.1,10,0}}) {
    bool rejected=false;
    try {ObservedMap invalid(params[0],params[1],size_t(params[2]));}
    catch(const std::invalid_argument &) {rejected=true;}
    assert(rejected);
  }
}
