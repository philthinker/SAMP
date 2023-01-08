cmake_minimum_required(VERSION 3.4)

# Project name
project(Grape)

# Set standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
list(APPEND CMAKE_PREFIX_PATH ~/Github/YOGURT/libfranka/build) # Change to your libfranka build path
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)

# Add libraries
add_subdirectory(LACTIC)
add_subdirectory(MILK)
add_subdirectory(TEA)

# Set variable

set(EigenDRINKS
    Toys_Grape6
)


# Add executable files and link libraries

foreach(drink ${EigenDRINKS})
    add_executable(${drink} ${drink}.cpp)
    target_link_libraries(${drink} LACTIC MILK TEA Franka::Franka Eigen3::Eigen)
endforeach(drink ${EigenDRINKS})
