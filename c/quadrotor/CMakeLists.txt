cmake_minimum_required (VERSION 2.6)
include(ExternalProject)

project(quadrotor)

set(PROJECT_SOURCE_DIR src)

include_directories("${PROJECT_SOURCE_DIR}")

add_library(quadrotor
	src/quadrotor/array.cpp
	src/quadrotor/attitude.cpp
	src/quadrotor/brain.cpp
	src/quadrotor/command.cpp
	src/quadrotor/plant.cpp
	src/quadrotor/position.cpp
	src/quadrotor/quadrotor.cpp
	src/quadrotor/telem.cpp
	src/quadrotor/fda.cpp
)

#configure_file("${PROJECT_SOURCE_DIR}/quadrotor/config.h.in" "${PROJECT_SOURCE_DIR}/math/config.h")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_DEBUG -g -std=c++0x -Wall -Werror -Wno-unused-local-typedefs -Wno-unknown-pragmas -rdynamic -pthread")
set(CMAKE_INSTALL_PREFIX $ENV{HOME}/usr)

install(TARGETS quadrotor DESTINATION lib)

install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/array.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/attitude.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/except.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/quadrotor.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/brain.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/plant.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/position.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/fda.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/command.h DESTINATION		include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/telem.h DESTINATION		include/quadrotor/)

#project(test_array)
#add_executable(test_array test/array/main.cpp)
#target_link_libraries(test_array math)




