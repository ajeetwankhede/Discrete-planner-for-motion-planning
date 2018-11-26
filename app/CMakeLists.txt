add_executable(shell-app main.cpp Map.cpp RandomPlanner.cpp OptimalPlanner.cpp Output.cpp)
target_link_libraries(shell-app ${Boost_LIBRARIES})
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
)
