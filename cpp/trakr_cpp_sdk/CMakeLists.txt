cmake_minimum_required(VERSION 3.5)

PROJECT(trakr_cpp_sdk CXX)
SET(PACKAGE_NAME "trakr_cpp_sdk")
SET(PACKAGE_VERSION "1.0.0")
SET(PACKAGE_SO_VERSION "1.0.0")
SET(PACKAGE_DESCRIPTION "Trakr's SDK for Communicating over Socket in C++")
SET(PACKAGE_AUTHOR "Addverb Technologies <humanoid@addverb.com>")
SET(PACKAGE_URL "https://addverb.ai")

include(GNUInstallDirs)

SET(CMAKE_PREFIX_PATH "/usr")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET(INSTALL_INCLUDE_PATH ${CMAKE_PREFIX_PATH}/include/trakr_cpp_sdk)
SET(INSTALL_LIB_PATH ${CMAKE_PREFIX_PATH}/lib/trakr_cpp_sdk)

SET(TRAKR_CPP_SDK_BUILD_EXAMPLES ON)

find_package(Eigen3 REQUIRED)

############################################################
#################### build and install #####################
############################################################

# source files
FILE(GLOB SRC src/*.cpp)

# import the compiled static libraries
FILE(GLOB TRAKR_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/amd64/*.a)

# installing static libraries
SET(TRAKR_LIBRARIES_DIR ${INSTALL_LIB_PATH}/lib/amd64)
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
target_link_directories(trakr_cpp_sdk PUBLIC 
  $<BUILD_INTERFACE:${TRAKR_LIBRARIES}>
  $<INSTALL_INTERFACE:${INSTALL_TRAKR_LIBRARIES}>
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

############################################################
######################### examples #########################
############################################################

if (TRAKR_CPP_SDK_BUILD_EXAMPLES)
    # compile trakr examples
    SET(EXAMPLE_NAMES
        basic
        keyboard
    )

    FOREACH(ELEMENT ${EXAMPLE_NAMES})
        ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
        TARGET_INCLUDE_DIRECTORIES(${ELEMENT} PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/trakr_cpp_sdk)
        TARGET_LINK_LIBRARIES(${ELEMENT} trakr_cpp_sdk ${TRAKR_LIBRARIES})
    ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})
endif(TRAKR_CPP_SDK_BUILD_EXAMPLES)

##
##   end of file
##
