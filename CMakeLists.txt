cmake_minimum_required(VERSION 3.5)
project(ur5_rg2_ign)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY urdf ur5_rg2/meshes DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
