cmake_minimum_required(VERSION 3.24) 
project( CS_548_Assignments)
set(CMAKE_PREFIX_PATH "D:/Software/Windows/PCL 1.14.1")
set(CMAKE_INSTALL_PREFIX .)

#####################################
# Enable testing
#####################################
enable_testing()
 
#####################################
# Require C++17
#####################################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#####################################
# Find necessary libraries
#####################################

include(FetchContent)

#####################################
# PCL
#####################################

find_package(PCL CONFIG REQUIRED) 
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

#####################################
# Get general sources
#####################################

include_directories("src/include" ${PCL_INCLUDE_DIRS})

file(GLOB GENERAL_SOURCES    
    "src/lib/*.cpp"  
    "src/lib/*.hpp"  
    "src/lib/*.h"
)

#####################################
# Create executables 
# and install targets
#####################################

set(ALL_LIBRARIES 
    ${PCL_LIBRARIES})

macro(CREATE_EXECUTABLE target)
    add_executable(${target} ${GENERAL_SOURCES} "./src/app/${target}.cpp")
    target_link_libraries(${target} PRIVATE ${ALL_LIBRARIES})
    install(TARGETS ${target} RUNTIME DESTINATION bin/${target})    
endmacro()

macro(CREATE_TEST target)
    add_executable(${target} ${GENERAL_SOURCES} "./src/tests/${target}.cpp")
    target_link_libraries(${target} PRIVATE ${ALL_LIBRARIES})    
    add_test(NAME ${target} COMMAND ${target}) 
endmacro()

CREATE_EXECUTABLE(SimplePCL)
# CREATE_EXECUTABLE(Assign01)
CREATE_TEST(Test_Assign01)

