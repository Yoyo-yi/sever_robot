cmake_minimum_required(VERSION 3.0.0)
project(patrol VERSION 0.1.0 LANGUAGES C CXX)

# Enable C++11
# set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../)

# 手动find线程库
find_package(Threads REQUIRED)
# 
find_package(CURL REQUIRED)
set(CURL_LIBRARY ${CURL_LIBRARIES})
find_package(OpenSSL REQUIRED)

# 包含 wiringPi 库的查找路径
find_library(WIRINGPI_LIB wiringPi)
    # 检查 wiringPi 库是否存在
if(NOT WIRINGPI_LIB)
    message(FATAL_ERROR "wiringPi library not found!")
endif()

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

include_directories(
    ${CURL_INCLUDE_DIRS}
)

add_executable(patrol_11_27 src/test_11_27.cpp)
target_link_libraries(patrol_11_27
    ${WIRINGPI_LIB}
    ${CURL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    OpenSSL::Crypto
)
target_include_directories(patrol_11_27 PRIVATE 
    ${CMAKE_SOURCE_DIR}/src
)

