cmake_minimum_required(VERSION 3.0)

PROJECT(trakr_cpp_sdk CXX)
SET(PACKAGE_NAME "trakr_cpp_sdk")
SET(PACKAGE_VERSION "1.0.0")
SET(PACKAGE_SO_VERSION "1.0.0")
SET(PACKAGE_DESCRIPTION "Trakr's SDK for Communicating over Socket in C++")
SET(PACKAGE_AUTHOR "Addverb Technologies <humanoid@addverb.com>")
SET(PACKAGE_URL "https://addverb.ai")

include(GNUInstallDirs)
# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-pthread")

SET(CMAKE_PREFIX_PATH "/usr")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET(INSTALL_INCLUDE_PATH ${CMAKE_PREFIX_PATH}/include/trakr_cpp_sdk)
SET(INSTALL_LIB_PATH ${CMAKE_PREFIX_PATH}/lib/trakr_cpp_sdk)

SET(TRAKR_CPP_SDK_BUILD_EXAMPLES ON)
OPTION(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" ON)

find_package(Eigen3 REQUIRED)

############################################################
#################### build and install #####################
############################################################

# source files
FILE(GLOB SRC src/robot.cpp)

# check the system processor
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)|(x86_64)")
    message("-- Target platform is AMD/Intel x86_64")
    SET(LIB_TYPE amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(arm)|(ARM)|(aarch64)|(AARCH64)")
    message("-- Target platform is ARM")
    SET(LIB_TYPE aarch64)
else()
    message("-- Target platform is unknown: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# import the compiled static libraries
SET(TRAKR_LIBRARIES_BUILD_DIR "${PROJECT_SOURCE_DIR}/lib/${LIB_TYPE}")
FILE(GLOB TRAKR_LIBRARIES ${TRAKR_LIBRARIES_BUILD_DIR}/*.a)

# installing static libraries
SET(TRAKR_LIBRARIES_DIR "${INSTALL_LIB_PATH}/lib/${LIB_TYPE}")
INSTALL(
  FILES ${TRAKR_LIBRARIES}
  DESTINATION ${TRAKR_LIBRARIES_DIR}
)

# path to installed static libraries
FILE(GLOB INSTALL_TRAKR_LIBRARIES ${TRAKR_LIBRARIES_DIR}/*.a)

# compiling library
ADD_LIBRARY(trakr_cpp_sdk SHARED ${SRC})
INSTALL(
  TARGETS trakr_cpp_sdk
  EXPORT trakr_cpp_sdk
  LIBRARY DESTINATION ${INSTALL_LIB_PATH}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

# include directories - depending on build/install
target_include_directories(trakr_cpp_sdk PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/trakr_cpp_sdk>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_PATH}/include>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_PATH}/include/trakr_cpp_sdk>
)

# link targets - depending on install
target_link_libraries(trakr_cpp_sdk PUBLIC
  ${TRAKR_LIBRARIES}
  ${INSTALL_TRAKR_LIBRARIES}
)

# setting library version
SET_TARGET_PROPERTIES(
  trakr_cpp_sdk
  PROPERTIES
  SOVERSION ${PACKAGE_SO_VERSION}
  )

# headers
FILE(GLOB TRAKR_HEADERS include/trakr_cpp_sdk/*.h)
INSTALL(
  FILES include/robot.h
  DESTINATION ${INSTALL_INCLUDE_PATH}/include)
INSTALL(
  DIRECTORY include/trakr_cpp_sdk
  DESTINATION ${INSTALL_INCLUDE_PATH}/include
  FILES_MATCHING PATTERN "*.h")

# export
INSTALL(
  FILES cmake/trakr_cpp_sdk-config.cmake 
  DESTINATION ${INSTALL_LIB_PATH})
INSTALL(
  EXPORT trakr_cpp_sdk
  DESTINATION ${INSTALL_LIB_PATH}
)

if(BUILD_PYTHON_BINDINGS)
    message("-- Python bindings will be built")
    cmake_policy(SET CMP0057 NEW)

    set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(PYTHON_TRAKR_PACKAGE_DIR "${PROJECT_SOURCE_DIR}/../../python")

    set(PYBIND11_FINDPYTHON ON)
    set(PYBIND11_PYTHON_VERSION 3.8)
    find_package(Python 3.8...<3.9 COMPONENTS Interpreter Development Development.SABIModule REQUIRED)
    find_package(pybind11)
    if(${pybind11_FOUND})
      pybind11_add_module(trakr_python_sdk MODULE src/bindings.cpp)
      target_include_directories(trakr_python_sdk PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/trakr_cpp_sdk>
        $<INSTALL_INTERFACE:${INSTALL_INCLUDE_PATH}/include>
        $<INSTALL_INTERFACE:${INSTALL_INCLUDE_PATH}/include/trakr_cpp_sdk>
      )
      target_link_libraries(trakr_python_sdk PRIVATE 
        ${TRAKR_LIBRARIES}
        ${INSTALL_TRAKR_LIBRARIES}
      )
      install(TARGETS trakr_python_sdk
        DESTINATION "${PYTHON_TRAKR_PACKAGE_DIR}/trakr_sdk/lib"
      )
    else()
      message(WARNING "-- Python Bindings enabled but pybind11 not found.\n Try installing with 'python3.8 -m pip install pybind11'.\n Skipping Python bindings.")
    endif()
else()
    message("-- Skipping Python bindings")
endif()

############################################################
######################### examples #########################
############################################################

if (TRAKR_CPP_SDK_BUILD_EXAMPLES)
    # compile trakr examples
    SET(EXAMPLE_NAMES
        basic
        keyboard
        lowlevel
        qwerty_control
    )

    FOREACH(ELEMENT ${EXAMPLE_NAMES})
        ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
        TARGET_INCLUDE_DIRECTORIES(${ELEMENT} PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/trakr_cpp_sdk)
        TARGET_LINK_LIBRARIES(${ELEMENT} trakr_cpp_sdk)
    ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})
endif(TRAKR_CPP_SDK_BUILD_EXAMPLES)

##
##   end of file
##
