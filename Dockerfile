ARG ROS_IMAGE=ros:rolling-ros-base
FROM ${ROS_IMAGE}

ARG NAVMAP_REF=rolling
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-rolling-pcl-conversions \
    ros-rolling-pcl-ros \
    ros-rolling-rviz-common \
    ros-rolling-rviz-rendering \
    ros-rolling-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true
RUN rosdep update || true

WORKDIR /opt/navmap_ws/src
RUN git clone --branch ${NAVMAP_REF} https://github.com/EasyNavigation/NavMap.git

RUN mkdir -p /opt/navmap_ws/src/bag_to_navmap_helper/src

RUN cat > /opt/navmap_ws/src/bag_to_navmap_helper/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bag_to_navmap_helper</name>
  <version>0.1.0</version>
  <description>Internal helper to write binary NavMap files.</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>navmap_core</depend>
  <depend>navmap_ros</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

RUN cat > /opt/navmap_ws/src/bag_to_navmap_helper/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(bag_to_navmap_helper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)

add_executable(obj_to_navmap src/obj_to_navmap.cpp)
target_link_libraries(obj_to_navmap navmap_core::navmap_core navmap_ros::navmap_ros)

install(TARGETS obj_to_navmap
  DESTINATION lib/${PROJECT_NAME})

ament_package()
EOF

RUN cat > /opt/navmap_ws/src/bag_to_navmap_helper/src/obj_to_navmap.cpp <<'EOF'
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/navmap_io.hpp"

namespace fs = std::filesystem;

struct ObjMesh {
  std::vector<Eigen::Vector3f> vertices;
  std::vector<std::array<uint32_t, 3>> triangles;
};

static std::string trim(const std::string & s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return "";
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

static std::vector<std::string> split_ws(const std::string & line) {
  std::istringstream iss(line);
  std::vector<std::string> out;
  std::string token;
  while (iss >> token) out.push_back(token);
  return out;
}

static std::optional<int> parse_int_sv(std::string_view sv) {
  int value = 0;
  auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), value);
  if (ec != std::errc() || ptr != sv.data() + sv.size()) return std::nullopt;
  return value;
}

static std::optional<uint32_t> parse_obj_index(const std::string & token, size_t vertex_count) {
  const auto slash = token.find('/');
  const std::string idx_str = (slash == std::string::npos) ? token : token.substr(0, slash);
  if (idx_str.empty()) return std::nullopt;

  auto idx_opt = parse_int_sv(idx_str);
  if (!idx_opt.has_value()) return std::nullopt;
  int idx = *idx_opt;
  if (idx == 0) return std::nullopt;

  if (idx > 0) {
    int zero_based = idx - 1;
    if (static_cast<size_t>(zero_based) >= vertex_count) return std::nullopt;
    return static_cast<uint32_t>(zero_based);
  }

  int zero_based = static_cast<int>(vertex_count) + idx;
  if (zero_based < 0 || static_cast<size_t>(zero_based) >= vertex_count) return std::nullopt;
  return static_cast<uint32_t>(zero_based);
}

static bool is_degenerate(
  const Eigen::Vector3f & a,
  const Eigen::Vector3f & b,
  const Eigen::Vector3f & c)
{
  return (b - a).cross(c - a).norm() < 1e-9f;
}

static ObjMesh load_obj(const fs::path & path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open OBJ: " + path.string());
  }

  ObjMesh mesh;
  std::string line;

  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;

    auto tokens = split_ws(line);
    if (tokens.empty()) continue;

    if (tokens[0] == "v") {
      if (tokens.size() < 4) throw std::runtime_error("Malformed OBJ vertex line");
      mesh.vertices.emplace_back(
        std::stof(tokens[1]),
        std::stof(tokens[2]),
        std::stof(tokens[3]));
      continue;
    }

    if (tokens[0] == "f") {
      if (tokens.size() < 4) continue;

      std::vector<uint32_t> face_indices;
      for (size_t i = 1; i < tokens.size(); ++i) {
        auto idx = parse_obj_index(tokens[i], mesh.vertices.size());
        if (!idx.has_value()) throw std::runtime_error("Invalid OBJ face index");
        face_indices.push_back(*idx);
      }

      for (size_t i = 1; i + 1 < face_indices.size(); ++i) {
        uint32_t a = face_indices[0];
        uint32_t b = face_indices[i];
        uint32_t c = face_indices[i + 1];
        if (a == b || b == c || a == c) continue;
        if (is_degenerate(mesh.vertices[a], mesh.vertices[b], mesh.vertices[c])) continue;
        mesh.triangles.push_back({a, b, c});
      }
    }
  }

  if (mesh.vertices.empty()) throw std::runtime_error("OBJ has no vertices");
  if (mesh.triangles.empty()) throw std::runtime_error("OBJ has no valid triangles");
  return mesh;
}

int main(int argc, char ** argv) {
  if (argc < 3) {
    std::cerr << "Usage: obj_to_navmap INPUT.obj OUTPUT.navmap [SURFACE_NAME]\n";
    return 2;
  }

  try {
    fs::path input_path = argv[1];
    fs::path output_path = argv[2];
    std::string surface_name = (argc >= 4) ? argv[3] : "map";

    ObjMesh mesh = load_obj(input_path);

    navmap::NavMap nm;
    std::size_t surface_idx = nm.create_surface(surface_name);

    for (const auto & v : mesh.vertices) {
      nm.add_vertex({v.x(), v.y(), v.z()});
    }

    for (const auto & tri : mesh.triangles) {
      auto cid = nm.add_navcel(tri[0], tri[1], tri[2]);
      nm.add_navcel_to_surface(surface_idx, cid);
    }

    // Must call after all geometry modifications, before any queries.
    nm.rebuild_geometry_accels();

    auto parent = output_path.parent_path();
    if (!parent.empty()) {
      fs::create_directories(parent);
    }

    std::error_code ec;
    if (!navmap_ros::io::save_to_file(nm, output_path.string(), {}, &ec)) {
      throw std::runtime_error("save_to_file failed: " + ec.message());
    }

    std::cerr << "Wrote " << output_path << "\n";
    return 0;

  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
EOF

WORKDIR /opt/navmap_ws
RUN rosdep install --from-paths src --ignore-src -r -y && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-up-to bag_to_navmap_helper && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt /app/requirements.txt
RUN python3 -m pip install --no-cache-dir --break-system-packages -r /app/requirements.txt

COPY entrypoint.sh /app/entrypoint.sh
COPY bag_to_navmap.py /app/bag_to_navmap.py
RUN chmod +x /app/entrypoint.sh /app/bag_to_navmap.py

ENTRYPOINT ["/app/entrypoint.sh"]
