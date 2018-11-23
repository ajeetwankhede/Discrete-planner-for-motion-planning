set(GTEST_SHUFFLE 1)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

add_executable(
    cpp-test
    main.cpp
    MapTest.cpp
    RandomPlannerTest.cpp
    OptimalPlannerTest.cpp
    OutputTest.cpp
    
    ../app/Map.cpp
    ../app/RandomPlanner.cpp    
    ../app/OptimalPlanner.cpp
    ../app/Output.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
