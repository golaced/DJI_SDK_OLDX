cmake_minimum_required(VERSION 2.8)

project(DigitDetector)

find_package(OpenCV REQUIRED)

set(CMAKE_DEBUG_POSTFIX "d")

include_directories(${PROJECT_SOURCE_DIR}/src)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SOURCE_FILES)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SOURCE_FILES_LIBS)
list(REMOVE_ITEM SOURCE_FILES_LIBS ${PROJECT_SOURCE_DIR}/src/main.cpp)

# foreach(_file ${SOURCE_FILES})
#     message(STATUS ${_file})
# endforeach(_file ${SOURCE_FILES})
# foreach(_file ${SOURCE_FILES_LIBS})
#     message(STATUS ${_file})
# endforeach(_file ${SOURCE_FILES_LIBS})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES_LIBS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} blas)

add_executable(${PROJECT_NAME}_exe ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_exe ${OpenCV_LIBS} blas)

file(GLOB dd_headers ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
file(COPY ${dd_headers} DESTINATION "${CMAKE_BINARY_DIR}/include/DigitDetector")
file(GLOB dd_params ${CMAKE_CURRENT_SOURCE_DIR}/*.txt)
list(REMOVE_ITEM dd_params ${PROJECT_SOURCE_DIR}/CMakeLists.txt)
file(COPY ${dd_params} DESTINATION "${CMAKE_SOURCE_DIR}/config")

# install target

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_exe
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)
install(FILES ${dd_headers}
        DESTINATION include/DigitDetector
)