cmake_minimum_required (VERSION 2.6)
include(ExternalProject)

project(quadrotor)

set(PROJECT_SOURCE_DIR src)
set(CMAKE_CURRENT_SOURCE_DIR src)

include_directories("${PROJECT_SOURCE_DIR}")

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
		${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM
		)
endif(DOXYGEN_FOUND)


add_library(quadrotor
	src/quadrotor/array.cpp
	src/quadrotor/attitude.cpp
	src/quadrotor/brain.cpp
	src/quadrotor/command.cpp
	src/quadrotor/Input.cpp
	src/quadrotor/plant.cpp
	src/quadrotor/position.cpp
	src/quadrotor/quadrotor.cpp
	src/quadrotor/telem.cpp
	src/quadrotor/fda.cpp
	src/quadrotor/command/Stop.cpp
	src/quadrotor/ControlLaw/ControlLaw.cpp
	src/quadrotor/ControlLaw/Alpha.cpp
	src/quadrotor/ControlLaw/Jerk.cpp
	src/quadrotor/ControlLaw/Jounce.cpp
)

#configure_file("${PROJECT_SOURCE_DIR}/quadrotor/config.h.in" "${PROJECT_SOURCE_DIR}/math/config.h")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_DEBUG -g -std=c++0x -Wall -Werror -Wno-unused-local-typedefs -Wno-unknown-pragmas -rdynamic -pthread")
set(CMAKE_INSTALL_PREFIX $ENV{HOME}/usr)

install(TARGETS quadrotor DESTINATION lib)

install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/array.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/attitude.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/except.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/quadrotor.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/brain.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/plant.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/position.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/fda.h DESTINATION				include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/command.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/Input.hpp DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/command/Stop.hpp	DESTINATION		include/quadrotor/command)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/telem.h DESTINATION			include/quadrotor/)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/ControlLaw/ControlLaw.h	DESTINATION	include/quadrotor/ControlLaw)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/ControlLaw/Alpha.h	DESTINATION	include/quadrotor/ControlLaw)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/ControlLaw/Jerk.h	DESTINATION		include/quadrotor/ControlLaw)
install(FILES ${PROJECT_SOURCE_DIR}/quadrotor/ControlLaw/Jounce.h	DESTINATION	include/quadrotor/ControlLaw)

#project(test_array)
#add_executable(test_array test/array/main.cpp)
#target_link_libraries(test_array math)


