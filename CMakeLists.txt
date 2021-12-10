cmake_minimum_required(VERSION 3.20)
project(ParticleFilter)

set(CMAKE_CXX_STANDARD 23)

add_executable(ParticleFilter
                particlefilt.cpp
                robot.cpp
                robotfield.cpp
                robot.h
                robotfield.h)
