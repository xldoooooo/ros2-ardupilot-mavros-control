/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef ODIN_CALIB_PATH_H
#define ODIN_CALIB_PATH_H

#include <cstdlib>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

// Single source of truth for the calibration file location, shared by the
// writer (host_sdk_sample) and every reader (cloud_reprojection, pcd2depth).
// Header-only inline functions avoid multiple-definition across translation units.
namespace odin_ros_driver {

// Resolve the same local selector used by shell and Python entry points.
inline std::string GetProjectConfigDir(const std::string& start) {
    auto root = std::filesystem::path(start);
    while (!std::filesystem::exists(root / "project_configuration.py")) {
        const auto parent = root.parent_path();
        if (parent.empty() || parent == root) {
            throw std::runtime_error("Project configuration root not found from " + start);
        }
        root = parent;
    }
    std::string aircraft = "simulation";
    std::ifstream selector(root / "config/aircraft.local.env");
    std::string line;
    while (std::getline(selector, line)) {
        line = line.substr(0, line.find('#'));
        const auto separator = line.find('=');
        if (separator == std::string::npos) continue;
        auto key = line.substr(0, separator);
        key.erase(std::remove_if(key.begin(), key.end(), [](unsigned char c) {
            return std::isspace(c);
        }), key.end());
        if (key != "AIRCRAFT_ID") continue;
        aircraft = line.substr(separator + 1);
        aircraft.erase(std::remove_if(aircraft.begin(), aircraft.end(), [](unsigned char c) {
            return std::isspace(c) || c == '\'' || c == '"';
        }), aircraft.end());
    }
    const auto config = root / "config/aircraft" / aircraft;
    if (aircraft.empty() || aircraft.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos ||
            !std::filesystem::is_directory(config)) {
        throw std::runtime_error("Invalid aircraft configuration: " + config.string());
    }
    return config.string();
}

// Resolve the writable runtime directory that stores calib.yaml.
// Project launchers export ODIN_CALIB_DIR; direct binaries use the same selector.
inline std::string GetOdinRuntimeDir() {
    if (const char* custom = std::getenv("ODIN_CALIB_DIR")) {
        return std::string(custom);
    }
    return GetProjectConfigDir(std::filesystem::read_symlink("/proc/self/exe").string()) +
        "/odin/calibration";
}

// Canonical calib.yaml path shared by writer and all readers.
inline std::string GetOdinCalibFile() {
    return GetOdinRuntimeDir() + "/calib.yaml";
}

}  // namespace odin_ros_driver

#endif  // ODIN_CALIB_PATH_H
