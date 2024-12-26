cmake_minimum_required(VERSION 3.14)
project(autoware_nlp_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(cgmres REQUIRED)

set(NLP_INTERFACE_LIB_SRC
  src/solvers/cgmres_solver.cpp
)

# Header files
set(NLP_INTERFACE_LIB_HEADERS
  include/nlp_interface/base/solver_interface.hpp
  include/nlp_interface/solvers/cgmres_solver.hpp
  include/nlp_interface/types/cgmres_parameters.hpp
  include/nlp_interface/types/common_parameters.hpp
  include/nlp_interface/types/paramters.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${NLP_INTERFACE_LIB_SRC}
  ${NLP_INTERFACE_LIB_HEADERS}
)

# target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast -Wno-error=useless-cast)
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}"
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  cgmres
)

ament_auto_package(INSTALL_TO_SHARE
  design
)
