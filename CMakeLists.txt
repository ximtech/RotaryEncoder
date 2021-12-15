cmake_minimum_required(VERSION 3.20)
project(ROTARY_ENCODER VERSION 1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)

include(cmake/CPM.cmake)

CPMAddPackage(
        NAME DWTDelay
        GITHUB_REPOSITORY ximtech/DWTDelay
        GIT_TAG origin/main)

set(ROTARY_ENCODER_DIRECTORY
        ${DWT_DELAY_DIRECTORY}
        ${ROTARY_ENCODER_SOURCE_DIR}
        ${ROTARY_ENCODER_SOURCE_DIR}/include
        CACHE STRING "Rotary encoder directories include to the main project" FORCE)

set(ROTARY_ENCODER_SOURCES
        ${DWT_DELAY_SOURCES}
        ${ROTARY_ENCODER_SOURCE_DIR}/RotaryEncoder.c
        ${ROTARY_ENCODER_SOURCE_DIR}/include/RotaryEncoder.h
        CACHE STRING "Rotary encoder source files include to the main project" FORCE)