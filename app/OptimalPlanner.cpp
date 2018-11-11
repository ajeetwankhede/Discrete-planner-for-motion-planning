/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    OptimalPlanner.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  OptimalPlanner class definition of methods.
 */

#include "OptimalPlanner.hpp"
#include <vector>
#include <utility>

using std::vector;
using std::pair;
using std::make_pair;

OptimalPlanner::OptimalPlanner() {
  // Initializing values to the attributes of Map class
  noOfNodes = 0;
  time = 0;
}

OptimalPlanner::~OptimalPlanner() {
  // Destructor stub
}

void OptimalPlanner::initMap() {

}

vector<pair<int, int> > OptimalPlanner::search(vector<vector<int> > worldState,
                                              pair<int, int> robotPose,
                                              pair<int, int> goalPose) {
  return path;
}
