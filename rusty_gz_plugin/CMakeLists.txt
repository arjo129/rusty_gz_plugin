cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(rusty_system)

find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS graphics)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-rendering6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})
find_package(sdformat12 REQUIRED)

find_package(std_msgs REQUIRED)

#============================================================================
# Corrosion
include(FetchContent)

FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.2.1 # Optionally specify a commit hash, version tag or branch here
)

FetchContent_MakeAvailable(Corrosion)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================
# Rust Business Logic
corrosion_import_crate(MANIFEST_PATH src/rust_system/Cargo.toml)

#============================================================================
# Wrapper stuff
add_library(RustySystem SHARED
  src/RustySystem.cc)

target_link_libraries(RustySystem PUBLIC
  rust_system
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-math${IGN_MATH_VER})

install(TARGETS RustySystem DESTINATION lib)
ament_package()