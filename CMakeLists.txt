cmake_minimum_required( VERSION 3.5.1 )

# Set project name.
project(lab-pose-estimation)

# Set executable name.
set(exe_name lab-pose-estimation)
  
# Make cmake first look for "Find<package>.cmake" functions generated by conan.
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

# Export commands for vscode.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required libraries.
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

# We use the "OpenCV_LIBS" variable for backward compatibility with the old lab setup.
if (NOT OpenCV_LIBS)
  set(OpenCV_LIBS "opencv::opencv")
endif()

# Add an executable target to the project with the specified source files.
add_executable(${exe_name}
  lab_pose_estimation.h
  lab_pose_estimation.cpp
  ar_example.h
  ar_example.cpp
  feature_utils.h
  feature_utils.cpp
  homography_pose_estimator.h
  homography_pose_estimator.cpp
  main.cpp
  moba_pose_estimator.h
  moba_pose_estimator.cpp
  camera_projection_measurement.h
  camera_projection_measurement.cpp
  plane_world_model.h
  plane_world_model.cpp
  pnp_pose_estimator.h
  pnp_pose_estimator.cpp
  pose_estimator.h
  scene_3d.h
  scene_3d.cpp
  )

# Specify libraries that will be linked with the executable target.
target_link_libraries(${exe_name}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Sophus::Sophus
  )

# Set properties for the executable target.
set_target_properties(${exe_name} PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 17
  )

# Define two groups of supported compilers.
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")

# Set compiler specific flags and definitions.
target_compile_options(${exe_name} PRIVATE
  "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wformat=2>>"
  "$<${msvc_cxx}:$<BUILD_INTERFACE:-W4>>"
  )
target_compile_definitions(${exe_name} PUBLIC
  "$<${msvc_cxx}:-D_USE_MATH_DEFINES>"
  )
