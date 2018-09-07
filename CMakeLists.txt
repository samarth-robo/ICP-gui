cmake_minimum_required(VERSION 2.8.11)

project(qtcreator_test)
set(CMAKE_CXX_STANDARD 11)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5Widgets REQUIRED )

find_package(VTK REQUIRED)
find_package(PCL 1.7.1 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES
  main.cpp
  pclviewer.cpp
  Vis.cpp
  poseestimator.cpp
  mesh_sample.cpp)
add_executable(${PROJECT_NAME} ${project_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
qt5_use_modules(${PROJECT_NAME} Widgets)

add_executable(grasp_processor
    grasp_processor_main.cpp
    grasp_processor.cpp
    grasp_processor.h
    poseestimator.cpp
    poseestimator.h
    mesh_sample.cpp)
target_link_libraries(grasp_processor ${PCL_LIBRARIES})
