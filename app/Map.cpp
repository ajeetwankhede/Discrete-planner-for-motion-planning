/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    Map.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/09/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  Map class definition of methods.
 */

#include <Map.hpp>
#include <map>
#include <vector>
#include <utility>

using std::map;
using std::vector;
using std::pair;
using std::make_pair;

Map::Map() {
  // Initializing values to the attributes of Map class
  length = -1;
  width = -1;
}

Map::~Map() {
  // Destructor stub
}

void showMap(vector<pair<int, int> > path) {

}

bool verifyNodes(map<pair<int, int>, int> visitedNodes, pair<int, int> node) {
  return false;
}

pair<int, int> action(int i, pair<int, int> currentNode) {
  return make_pair(0, 0);
}
