cmake_minimum_required(VERSION 2.8.4)

find_package(GTest REQUIRED)

#include_directories()

file(GLOB SRCS "./*.cc")

foreach(src ${SRCS})
  GET_FILENAME_COMPONENT(name_ ${src} NAME_WE)
  message(STATUS "src -->: ${src} : ${name_}")
  set(exec_name test_${PROJECT_NAME}_${name_})
  add_executable(${exec_name} ${src})
  add_test(NAME ${name_} COMMAND $<TARGET_FILE:${exec_name}>)
  set_property(TEST ${name_} APPEND PROPERTY LABELS ${PROJECT_SOURCE_DIR}/src)

  set(PRJ_LIB "${PROJECT_NAME}")

  target_link_libraries(
    ${exec_name}
    ${PRJ_LIB}
    ${GTEST_BOTH_LIBRARIES}
    pthread
    )
endforeach()

