cmake_minimum_required(VERSION 3.25)
project(pomdp)

set(CMAKE_CXX_STANDARD 17)
set(DESPOT "/home/bhuvanesh/CLionProjects/despot/src" CACHE PATH "Despot Path")
include_directories(src/main/out)
include_directories(src/main/out/jniHeaders)
include_directories(src/main/resources/gcc)
include_directories(target/classes/gcc)
include_directories(/usr/lib/jvm/java-11-openjdk-amd64/include)
include_directories(/usr/lib/jvm/java-11-openjdk-amd64/include/linux)

file(GLOB DESPOT_INTERFACE "${DESPOT}/interface/*.cpp")
file(GLOB DESPOT_CORE "${DESPOT}/core/*.cpp")
file(GLOB DESPOT_SRC "${DESPOT}/*.cpp")
file(GLOB DESPOT_POMDPX "${DESPOT}/pomdpx/*.cpp")
file(GLOB DESPOT_SOLVER"${DESPOT}/solver/*.cpp")
file(GLOB DESPOT_UTIL"${DESPOT}/util/*.cpp")
file(GLOB DESPOT_UTIL_TINYXML"${DESPOT}/util/tinyxml/*.cpp")



add_executable(pomdp
        src/main/cpp/TemperatureSampler.cpp
        src/main/out/jniHeaders/com_jni_example_TemperatureSampler.h
        src/main/resources/gcc/functions-v1.c
        src/main/resources/gcc/functions-v2.c
        src/main/resources/gcc/functions.h
        target/classes/gcc/functions-v1.c
        target/classes/gcc/functions-v2.c
        target/classes/gcc/functions.h src/test/gcc/Test.cpp
        ${DESPOT_INTERFACE}
        ${DESPOT_CORE}
        ${DESPOT_SRC}
        ${DESPOT_POMDPX}
        ${DESPOT_SOLVER}
        ${DESPOT_UTIL}
        ${DESPOT_UTIL_TINYXML} src/main/cpp/TigerState.cpp src/main/cpp/TigerState.h src/main/cpp/AJAN_Agent/src/ajanPlanner.cpp src/main/cpp/AJAN_Agent/src/ajanAgent.cpp src/main/out/jniHeaders/de_dfki_asr_ajan_pluginsystem_mdpplugin_utils_POMDP_AJANPlanner.h src/main/cpp/AJAN_Agent/src/ajanWorld.cpp src/main/cpp/AJAN_Agent/src/ajanWorld.h)
