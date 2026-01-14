cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Thêm dòng này để trỏ tới thư mục chứa thư viện của bạn
list(APPEND EXTRA_COMPONENT_DIRS component)

project(PPG_Sensor)