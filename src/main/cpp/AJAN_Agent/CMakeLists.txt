cmake_minimum_required(VERSION 2.8.3)


include_directories(/usr/lib/jvm/java-11-openjdk-amd64/include)
include_directories(/usr/lib/jvm/java-11-openjdk-amd64/include/linux)
add_executable("${PROJECT_NAME}_AJAN_Agent"
        src/ajan_agent.cpp
        src/main.cpp
        ../drone/src/main.cpp src/com_ajan_POMDP_AJANPlanner.h)
target_link_libraries("${PROJECT_NAME}_AJAN_Agent"
        "${PROJECT_NAME}"
        )
install(TARGETS "${PROJECT_NAME}_AJAN_Agent"
        RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
        )