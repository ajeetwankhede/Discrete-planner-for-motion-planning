/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    RandomPlannerTest.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/23/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  Test stubs for class OptimalPlanner.
 */

#include <gtest/gtest.h>
#include <memory>
#include <map>
#include <vector>
#include <utility>
#include "OptimalPlanner.hpp"


std::shared_ptr<OptimalPlanner> OptimalPlannerTestObject;

using std::pair;
using std::make_pair;
using std::vector;

/**
 * @brief Check if initMap method works
 */
TEST(OptimalPlanner, initMapTest) {
  OptimalPlannerTestObject = std::make_shared<OptimalPlanner>();
  // Creating a map with no obstacles
  vector<vector<int> > worldState(4, vector<int>(4, 0));
  OptimalPlannerTestObject->M.world = worldState;
  OptimalPlannerTestObject->M.length = worldState[0].size();
  OptimalPlannerTestObject->M.width = worldState.size();
  // Call the initMap method
  OptimalPlannerTestObject->initMap();
  // Check if visitedNodes map is initialized
  EXPECT_FALSE(OptimalPlannerTestObject->visitedNodes.empty());
  // Check if parentCost map is initialized
  EXPECT_FALSE(OptimalPlannerTestObject->parentCost.empty());
  // Check if currentCost map is initialized
  EXPECT_FALSE(OptimalPlannerTestObject->currentCost.empty());
}

/**
 * @brief Check if search method works for a sample map
 */
TEST(OptimalPlanner, searchTest) {
  OptimalPlannerTestObject = std::make_shared<OptimalPlanner>();
  // Creating a map with no obstacles
  vector<vector<int> > worldState(4, vector<int>(4, 0));
  // Set start and goal pose
  pair<int, int> robotPose = make_pair(1, 1);
  pair<int, int> goalPose = make_pair(3, 1);
  vector < pair<int, int> > path;
  // Call the search method
  path = OptimalPlannerTestObject->search(worldState, robotPose, goalPose);
  vector < pair<int, int> > expectedPath;
  expectedPath.push_back(robotPose);
  expectedPath.push_back(make_pair(2, 1));
  expectedPath.push_back(goalPose);
  EXPECT_EQ(expectedPath, path);
}
