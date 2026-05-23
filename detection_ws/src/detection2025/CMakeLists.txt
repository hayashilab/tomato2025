cmake_minimum_required(VERSION 3.5)
project(detection2025)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# === Install launch files ===
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# === Install model files ===
install(DIRECTORY detection2025/models DESTINATION share/${PROJECT_NAME})

# === Install config files ===
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
