cmake_minimum_required(VERSION 3.0.2)
project(active_markers)

find_package( Boost COMPONENTS program_options thread REQUIRED )


set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")




find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)

add_executable(active_markers main.cpp
        src/utils/options.cpp
        include/utils/options.hpp
        src/camera.cpp
        include/camera.hpp
        src/event_reader.cpp
        include/event_reader.hpp
        include/utils/types.hpp
        src/utils/visualization.cpp
        include/utils/visualization.hpp
        src/utils/buffers.cpp
        include/utils/buffers.hpp src/markers.cpp include/markers.hpp src/runtime_manager.cpp include/runtime_manager.hpp src/detection_algorithm.cpp include/detection_algorithm.hpp include/utils/csvfile.h src/tracker.cpp include/tracker.hpp include/external/concurrentqueue.h src/logger_no_ros.cpp include/logger.hpp)

target_include_directories(active_markers SYSTEM PUBLIC ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
target_link_libraries( active_markers
                        ${OpenCV_LIBS}
                        yaml-cpp
                        MetavisionSDK::core
                        MetavisionSDK::driver
                        MetavisionSDK::ui
                        Boost::program_options
                        Boost::thread
                        ${catkin_LIBRARIES}
                         )

