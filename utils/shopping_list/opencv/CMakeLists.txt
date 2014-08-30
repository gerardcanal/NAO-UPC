cmake_minimum_required(VERSION 2.8)
project( NAO_BUY_LIST )

find_package( OpenCV REQUIRED )

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)


add_executable( nao_obj_detect src/main_detection.cpp )
add_executable( nao_obj_tracking src/main_tracking.cpp )

target_link_libraries( nao_obj_detect ${OpenCV_LIBS} )
target_link_libraries( nao_obj_tracking ${OpenCV_LIBS} )
