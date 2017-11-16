cmake_minimum_required(VERSION 2.8)

project(CorrelationTracker)

find_package(OpenCV REQUIRED)
find_package(dlib REQUIRED)

set(CMAKE_DEBUG_POSTFIX "d")

include_directories(${PROJECT_SOURCE_DIR}/CorrelationTracker ${PROJECT_SOURCE_DIR}/CorrelationTracker/dsst_tracker ${dlib_INCLUDE_DIRS})

aux_source_directory(${PROJECT_SOURCE_DIR}/CorrelationTracker SOURCE_FILES)
list(APPEND SOURCE_FILES ${PROJECT_SOURCE_DIR}/CorrelationTracker/dsst_tracker/dsst_tracker.cpp)

# foreach(_file ${SOURCE_FILES})
#     message(STATUS ${_file})
# endforeach(_file ${SOURCE_FILES})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${dlib_LIBRARIES})

file(GLOB ct_headers ${CMAKE_CURRENT_SOURCE_DIR}/CorrelationTracker/*.h)
file(COPY ${ct_headers} DESTINATION "${CMAKE_BINARY_DIR}/include/CorrelationTracker")

# install target

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)

install(FILES "${PROJECT_SOURCE_DIR}/CorrelationTracker/CorrelationTracker.h"
                "${PROJECT_SOURCE_DIR}/CorrelationTracker/correlationtracker_export.h"
        DESTINATION include/CorrelationTracker
)