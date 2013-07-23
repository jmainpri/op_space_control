cmake_minimum_required(VERSION 2.6)

project(op-space-control)

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Default mode is Release
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# ---------------------------------
# ---------------------------------
# ---------------------------------

# TODO fix use only one library
#include_directories( ${PROJECT_SOURCE_DIR}/libraries/KrisLibrary )
include_directories( /home/jmainpri/workspace/RobotSim/Library/KrisLibrary )
include_directories( /home/jmainpri/workspace/RobotSim )

# Check if the user is set up to parse URDFs
find_package( urdfdom QUIET )
if( urdfdom_FOUND )
   message(STATUS "Found urdfdom -- We will compile a URDF parser")

   include_directories( ${urdfdom_INCLUDE_DIRS} )
   link_libraries( ${urdfdom_LIBRARIES} )

   find_package( urdfdom_headers REQUIRED )
   include_directories( ${urdfdom_headers_INCLUDE_DIRS} )

# Remove when compiling with robot sim
#   find_package( Boost REQUIRED COMPONENTS system filesystem thread )
#   include_directories( ${BOOST_INCLUDE_DIRS} )
#   link_libraries( ${BOOST_LIBRARIES} )

else( urdfdom_FOUND )
   message(STATUS "Could NOT find urdfdom -- We will NOT compile a URDF parser")
endif( urdfdom_FOUND )

# ---------------------------------
# ---------------------------------
# ---------------------------------
    
file( GLOB lib_source "src/*.cpp" "src/*.h" )   
list( SORT lib_source )

#set( robot_sim_libs
#/home/jmainpri/workspace/RobotSim/Library/KrisLibrary/lib/libKrisLibrary.a)

find_package( OpenGL REQUIRED )
link_libraries( ${OPENGL_LIBRARIES} )

set( robot_sim_libs
/home/jmainpri/workspace/RobotSim/lib/libRobotSim.a
/home/jmainpri/workspace/RobotSim/Library/glui-2.36/src/lib/libglui.a
/home/jmainpri/workspace/RobotSim/Library/KrisLibrary/lib/libKrisLibrary.a
/home/jmainpri/workspace/RobotSim/Library/ode-0.11.1/ode/src/.libs/libode.so
/home/jmainpri/workspace/RobotSim/Library/tinyxml/libtinyxml.a
/home/jmainpri/workspace/devel/lib/libassimp.so )

# Define target
add_library( ${PROJECT_NAME} SHARED ${lib_source} ${robot_sim_libs} )

file( GLOB unit_tests_source "test/*.cpp" )
list( SORT unit_tests_source )


include_directories( src )
add_executable( test ${unit_tests_source} )
# TODO fix use only one library
#target_link_libraries( test ${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/libraries/KrisLibrary/lib/libKrisLibrary.a )
target_link_libraries( test ${PROJECT_NAME} ${robot_sim_libs} )

#enable_testing()
#message(STATUS "\n-- UNIT TEST: ")
#add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
#foreach(utest_src_file ${unit_tests_source})
#    get_filename_component(test_base ${utest_src_file} NAME_WE)
#    message(STATUS "Adding test ${test_base}")
#    add_executable(${test_base} ${utest_src_file})
#    target_link_libraries(${test_base} ${PROJECT_NAME})
#    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
#    add_custom_target(${test_base}.run ${test_base} ${ARGN})
#    add_dependencies(check ${test_base})
#endforeach(utest_src_file)

file( GLOB headers "src/drchubo_controller.h" )

install(FILES   ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/OpSpaceController)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
