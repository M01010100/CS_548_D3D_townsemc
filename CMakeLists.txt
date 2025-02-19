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
# doctest
#####################################

#find_package(doctest CONFIG REQUIRED)

FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.4.11
    GIT_SHALLOW 1
)

FetchContent_MakeAvailable(doctest)
include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

#####################################
# PCL
#####################################

find_package(PCL CONFIG REQUIRED) 
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

#####################################
# Get general sources
#####################################

include_directories("src/include" ${PCL_INCLUDE_DIRS} ${doctest_INCLUDE_DIR})

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
    ${PCL_LIBRARIES}
    doctest::doctest)

macro(CREATE_EXECUTABLE target)
    add_executable(${target} ${GENERAL_SOURCES} "./src/app/${target}.cpp")
    target_link_libraries(${target} PRIVATE ${ALL_LIBRARIES})
    install(TARGETS ${target} RUNTIME DESTINATION bin/${target})    
endmacro()

macro(CREATE_TEST target)
    add_executable(${target} ${GENERAL_SOURCES} "./src/tests/${target}.cpp")
    target_link_libraries(${target} PRIVATE ${ALL_LIBRARIES})    

    doctest_discover_tests(
        ${target} 
        TEST_PREFIX "${target}::"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

endmacro()

CREATE_EXECUTABLE(SimplePCL)
CREATE_EXECUTABLE(Assign01)
CREATE_TEST(Test_Assign01)
