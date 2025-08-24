#cmake最低版本要求
cmake_minimum_required(VERSION 3.4...3.18)
#任务名称指定
project(patrol_car)
#c++编译标准版本
set(CMAKE_CXX_STANDARD 17)
#g++编译选项
# set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread")

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS " -Wall -mcpu=native -flto -pthread")
add_compile_options(-pthread -g -ggdb -O0 -Wall -fstack-protector-all -march=native -fnon-call-exceptions)
#
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#下面的不用自己添加因为下面两个是cmake自己设置的默认变量
#设置本地头文件搜索路径
set(INCLUDE_PATH "/usr/local/include")
#设置本地库文件搜索路径
set(LIB_PATH "/usr/local/lib")
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#添加头文件，主要是头文件库文件搜索路径
#---------------------------------------------------------------------

#设置工程文件夹中的库文件搜索路径变量
#PROJECT_SOURCE_DIR是当前工作目录的地址
#意思就是有库文件可以再src目录中新建一个/lib文件夹放进去
set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")

#设置头文件所搜路径变量
#包括include config src tool文件夹
set(COMMON_INCLUDE_DIR  "${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/src")


#将上面设置的库文件夹路径添加进库文件所路径
link_directories(${COMMON_LIB_DIR})
#将上面设置的头文件夹搜索路径添加到头文件搜索路径
include_directories( ${COMMON_INCLUDE_DIR} )

#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#添加第三方库
#---------------------------------------------------------------------
#pkgconfig是用来做搜索库文件
#将这个包添加之后才能用pkg——serch_modle
find_package(PkgConfig)
#实现结果与find_package等价, 查找.pc文件, 查找路径一般是lib/pkgconfig, share/pkgconfig
#找到相应的包后就会定义<prefix>_INCLUDE_DIRS和其他的这类变量
#glib库是c语言的基础库
pkg_search_module(GLIB REQUIRED glib-2.0)
#包含GLIB包的头文件目录
include_directories(${GLIB_INCLUDE_DIRS})

#opencv
find_package( OpenCV REQUIRED )    
#添加头文件搜索路径
include_directories( ${OpenCV_INCLUDE_DIRS} )

#seral是串口库
pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})

#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
set(ICAR_PROJECT_NAME "main")
#set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/ICar.cpp)
set(ICAR_PROJECT_SOURCES 
    ${PROJECT_SOURCE_DIR}/src/ICar.cpp
    ${PROJECT_SOURCE_DIR}/src/ImagePrc.cpp
    ${PROJECT_SOURCE_DIR}/src/Control.cpp
    ${PROJECT_SOURCE_DIR}/src/Pid.cpp
    ${PROJECT_SOURCE_DIR}/src/SpeedCommand2.cpp
)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME} ${PPNC_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME} ${ONNX_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${ICAR_PROJECT_NAME} pthread )
target_link_libraries(${ICAR_PROJECT_NAME} ${SERIAL_LIBRARIES})


