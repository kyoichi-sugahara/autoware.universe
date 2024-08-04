cmake_minimum_required(VERSION 3.14)
project(nlp_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

find_package(cgmres REQUIRED)

set(NLP_INTERFACE_LIB_SRC
  src/nlp_interface.cpp
)

set(QP_INTERFACE_LIB_HEADERS
  include/nlp_interface/nlp_interface.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${NLP_INTERFACE_LIB_SRC}
  ${QP_INTERFACE_LIB_HEADERS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast -Wno-error=useless-cast)

target_include_directories(nlp_interface
  SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}"
)

ament_target_dependencies(nlp_interface
  Eigen3
)

ament_auto_package(INSTALL_TO_SHARE
)
