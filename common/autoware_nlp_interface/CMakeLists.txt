cmake_minimum_required(VERSION 3.14)
project(autoware_nlp_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

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

add_library(${PROJECT_NAME} SHARED
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

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Eigen3::Eigen
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY design
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)
