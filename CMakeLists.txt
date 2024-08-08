cmake_minimum_required(VERSION 3.10)

# 项目名称
project(MotorControllerProject)

# 指定 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 libutilities 库
find_library(LIB_U libutilities.so ${CMAKE_SOURCE_DIR})

# 查找 SDL2 库
# find_package(SDL2 REQUIRED)
# include_directories(${SDL2_INCLUDE_DIRS})

# 包含头文件目录
include_directories(${PROJECT_SOURCE_DIR})
# include_directories(${PROJECT_SOURCE_DIR}/include)

# 添加库文件目录
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 添加应用程序的可执行文件
add_executable(MotorControllerExecutable run.cpp MotorController.cpp MotorRunner.cpp)

# 添加测试程序的可执行文件
add_executable(DeepMotor DeepMotor.cpp MotorController.cpp MotorRunner.cpp)

# 链接需要的库
target_link_libraries(MotorControllerExecutable PRIVATE ${LIB_U} -lpthread -llcm usbcan-4e)
target_link_libraries(DeepMotor PRIVATE ${LIB_U} -lpthread -llcm usbcan-4e)
# target_link_libraries(test PRIVATE ${LIB_U} -lpthread -llcm controlcan)

