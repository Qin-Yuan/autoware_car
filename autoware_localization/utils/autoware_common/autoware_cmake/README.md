# autoware_cmake

This package provides CMake scripts for Autoware.

## Usage

### autoware_package.cmake

Call `autoware_package()` before defining build targets, which will set common options for Autoware.

```cmake
cmake_minimum_required(VERSION 3.5)
project(package_name)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(...)
```
