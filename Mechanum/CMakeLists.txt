cmake_minimum_required(VERSION 3.19)

project(Mechanum VERSION 0.1 LANGUAGES CXX)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/Libraries/SDL3")

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets SerialPort Network)
find_package(SDL3 REQUIRED CONFIG)
#find_package(SerialPort)


set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        ${TS_FILES}
)

add_executable(Mechanum ${PROJECT_SOURCES})

target_link_libraries(Mechanum
    PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::Network
    SDL3::SDL3
    Qt6::SerialPort
)

set_target_properties(Mechanum PROPERTIES
    ${BUNDLE_ID_OPTION}
    WIN32_EXECUTABLE TRUE
)

#include(GNUInstallDirs)
install(TARGETS Mechanum
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/Mecanum.png" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
