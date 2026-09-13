cmake_minimum_required(VERSION 3.24)

project(INS LANGUAGES CXX)

add_library(eskf STATIC
    "src/eskf.cpp"
)

target_compile_features(eskf PUBLIC cxx_std_20)

set_target_properties(eskf PROPERTIES
    CXX_EXTENSIONS OFF
)

# Eigen 5 dependency configuration 
include(FetchContent)

set(EIGEN_VERSION 5.0.1)
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG ${EIGEN_VERSION}
    GIT_SHALLOW TRUE                                       # creates a shallow clone
    FIND_PACKAGE_ARGS ${EIGEN_VERSION}                     # checks for this version or newer, otherwise installs it
)
FetchContent_MakeAvailable(Eigen3)


target_link_libraries(eskf 
    PUBLIC 
        Eigen3::Eigen
)

target_include_directories(eskf
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)