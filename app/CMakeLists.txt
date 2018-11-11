add_executable(shell-app main.cpp Map.cpp RandomPlanner.cpp OptimalPlanner.cpp Output.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
