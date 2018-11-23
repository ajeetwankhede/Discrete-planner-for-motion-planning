/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    MapTest.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  Test stubs for class Map.
 */

#include <gtest/gtest.h>
#include <map>
#include <utility>
#include <memory>
#include <Map.hpp>

std::shared_ptr<Map> mapTestObject;

using std::make_pair;
using std::pair;
using std::map;

/**
 * @brief Check if verifyNodes method works
 */
TEST(Map, verifyNodesTest) {
  mapTestObject = std::make_shared<Map>();
  map<pair<int, int>, int> visitedNodes;
  pair<int, int> node;
  node = make_pair(1, 1);
  visitedNodes[node] = 1;
  EXPECT_FALSE(mapTestObject->verifyNodes(visitedNodes, node));
  node = make_pair(2, 2);
  visitedNodes[node] = 0;
  EXPECT_TRUE(mapTestObject->verifyNodes(visitedNodes, node));
}

/**
 * @brief Check if action method works
 */
TEST(Map, actionTest) {
  mapTestObject = std::make_shared<Map>();
  int i = 1;
  pair<int, int> currentNode = make_pair(5, 5);
  // Run all the four actions and verify that they give the desired results
  while (i < 5) {
    EXPECT_EQ(make_pair(6, 5), mapTestObject->action(i, currentNode));
    EXPECT_EQ(make_pair(5, 6), mapTestObject->action(i, currentNode));
    EXPECT_EQ(make_pair(4, 5), mapTestObject->action(i, currentNode));
    EXPECT_EQ(make_pair(5, 4), mapTestObject->action(i, currentNode));
    i++;
  }
}
