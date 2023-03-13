if (DEFINED ENV{MICRO_ROS_PICO} AND (NOT MICRO_ROS_PICO))
    set(MICRO_ROS_PICO $ENV{MICRO_ROS_PICO} CACHE PATH "Path to micro ros raspberry pico library")
    message("Using MICRO_ROS_PICO from environment ('${MICRO_ROS_PICO}')")
endif ()

link_directories("${MICRO_ROS_PICO}")
include_directories("${MICRO_ROS_PICO}/include")
