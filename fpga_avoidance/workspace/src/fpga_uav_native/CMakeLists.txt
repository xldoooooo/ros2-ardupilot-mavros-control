# Build the optional original host acceleration kernels; no flight-control linkage.
cmake_minimum_required(VERSION 3.16)
project(fpga_uav_native)

find_package(ament_cmake REQUIRED)

# Host-only acceleration. ABI is 3301 and the FPGA firmware/RTL/protocol are unchanged.
# Adds exact CPU kernels for clearance, collision, spine, QUBO/payload and dense retiming.
add_library(fpga_fastpath_native SHARED src/native_fastpath.cpp)
target_compile_features(fpga_fastpath_native PRIVATE cxx_std_17)
target_compile_options(fpga_fastpath_native PRIVATE -O3 -DNDEBUG)
set_target_properties(fpga_fastpath_native PROPERTIES OUTPUT_NAME fpga_fastpath_native)

install(TARGETS fpga_fastpath_native
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
