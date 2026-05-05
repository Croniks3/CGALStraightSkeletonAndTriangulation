cmake_minimum_required(VERSION 3.25)
project(CGALBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# путь к твоему CGAL (ты его уже ставил)
set(CMAKE_PREFIX_PATH "F:/C++Libraries/CGAL/x64-windows")

find_package(CGAL CONFIG REQUIRED)

add_library(CGALBridge SHARED
    src/CGALbridge.h
    src/CGALbridge.cpp
)

target_compile_definitions(CGALBridge PRIVATE CGALBRIDGE_EXPORTS)

target_include_directories(CGALBridge PUBLIC src)

target_link_libraries(CGALBridge PRIVATE CGAL::CGAL)

add_executable(TestApp
    test.cpp
)

target_include_directories(TestApp PRIVATE src)

target_link_libraries(TestApp PRIVATE CGALBridge)