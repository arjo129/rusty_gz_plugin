cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(rusty_system)

find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-sim7 REQUIRED COMPONENTS gui)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-gui7 REQUIRED)
set(GZ_GUI_VER ${gz-gui7_VERSION_MAJOR})

find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

find_package(gz-sensors7 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})
set(GZ_SENSORS gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER})

find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

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
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(TARGETS RustySystem DESTINATION lib)
ament_package()
