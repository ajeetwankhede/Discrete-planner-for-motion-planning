/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    RandomPlannerTest.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/23/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  Test stubs for class RandomPlanner.
 */

#include <gtest/gtest.h>
#include <memory>
#include <map>
#include <vector>
#include <utility>
#include "RandomPlanner.hpp"

std::shared_ptr<RandomPlanner> randomPlannerTestObject;

using std::pair;
using std::make_pair;
using std::vector;

/**
 * @brief Check if initMap method works
 */
TEST(RandomPlanner, initMapTest) {
  randomPlannerTestObject = std::make_shared<RandomPlanner>();
  // Creating a map with no obstacles
  vector<vector<int> > worldState(4, vector<int>(4, 0));
  randomPlannerTestObject->M.world = worldState;
  randomPlannerTestObject->M.length = worldState[0].size();
  randomPlannerTestObject->M.width = worldState.size();
  // Call the initMap method
  randomPlannerTestObject->initMap();
  // Check if visitedNodes is initialized
  EXPECT_FALSE(randomPlannerTestObject->visitedNodes.empty());
  // Check if obstacleSpace is initialized
  EXPECT_FALSE(randomPlannerTestObject->obstacleSpace.empty());
}

/**
 * @brief Check if search method works for a sample map without obstacles
 */
TEST(RandomPlanner, searchTest) {
  randomPlannerTestObject = std::make_shared<RandomPlanner>();
  // Creating a map with no obstacles
  vector<vector<int> > worldState(4, vector<int>(4, 0));
  // Set start and goal pose
  pair<int, int> robotPose = make_pair(1, 1);
  pair<int, int> goalPose = make_pair(3, 1);
  vector<pair<int, int> > path;
  randomPlannerTestObject->time = 10.0;
  randomPlannerTestObject->maxStepNumber = 200;
  // Call the search method
  path = randomPlannerTestObject->search(worldState, robotPose, goalPose);
  EXPECT_FALSE(path.empty());
}
