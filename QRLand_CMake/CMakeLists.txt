CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
project(QRLand)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)
Message(CMAKE_BUILD_TYPE:${CMAKE_BUILD_TYPE})
find_package(OpenCV REQUIRED)
#find_package(aruco REQUIRED)
#find_package(serial REQUIRED)

Message(OpenCV:${OpenCV_VERSION})

AUX_SOURCE_DIRECTORY(./src SRC_LIST)

add_executable(QRLand ${SRC_LIST})
TARGET_LINK_LIBRARIES(QRLand ${OpenCV_LIBS} aruco serial)