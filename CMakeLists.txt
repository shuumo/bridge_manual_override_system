cmake_minimum_required(VERSION 3.16)

project(group_mm_host VERSION 0.0.1 DESCRIPTION "Host for the Bridge")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(QT_MAJOR_VERSION 6)
find_package(Qt${QT_MAJOR_VERSION} REQUIRED COMPONENTS Widgets Network)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(${PROJECT_NAME}
    include/app.hpp
    include/mainwindow.hpp
    include/connection_manager.hpp
    include/state_machine.hpp
    include/heartbeat.hpp
    include/types.hpp
    src/app.cpp
    src/mainwindow.cpp
    src/connection_manager.cpp
    src/state_machine.cpp
    src/heartbeat.cpp
    src/main.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt${QT_MAJOR_VERSION}::Widgets
        Qt${QT_MAJOR_VERSION}::Network
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME_DEBUG "${PROJECT_NAME}d"
    OUTPUT_NAME_RELEASE "${PROJECT_NAME}"
)

