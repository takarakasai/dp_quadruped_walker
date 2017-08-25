cmake_minimum_required(VERSION 2.8.4)

project(dp_quadruped_walker)

include("dplink.cmake" OPTIONAL)

# please add option() here
# option(<VARIABLE> <DECSCRIPTION> <ON or OFF>)
# if(VARIABLE)
#   add_definitions(-DEXAMPLE)
# endif()


## currently using target_include_directories
#include_directories(${PROJECT_SOURCE_DIR}/h/
#                   ${depend_prj_SOURCE_DIR}/h/
#                    )

file(GLOB SRCS "src/*.cc"
#              "src/OWN_SOURCE_DIR/*.cc"
   )

if(WITH_GCOV)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

# add own dependencies
#list(APPEND dp_quadruped_walker_DEPENDENCIES "<$OWN_DEPENDENCIES>")

add_library     (${PROJECT_NAME} SHARED ${SRCS})

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/h
  #$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/h>
  #$<INSTALL_INTERFACE:h/${PROJECT_NAME}>
  )

if(dp_quadruped_walker_DEPENDENCIES)
  add_dependencies(${PROJECT_NAME} ${dp_quadruped_walker_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME} ${dp_quadruped_walker_DEPENDENCIES})
endif()


if(${UNIX})
  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/h/${PROJECT_NAME} DESTINATION include)
elseif(${MINGW})
  install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${INSTALL_DIR})
endif()

option(TEST_${PROJECT_NAME} "test for ${PROJECT_NAME}" OFF)
if(TEST_${PROJECT_NAME} OR TEST_ALL)
  add_subdirectory(test)
endif()

