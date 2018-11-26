/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    main.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/09/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  main source file.
 */

#include <iostream>
#include <vector>
#include <utility>
#include "RandomPlanner.hpp"
#include "OptimalPlanner.hpp"

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::pair;
using std::make_pair;

int main() {
  // Creating a map with no obstacles
  vector<vector<int> > worldState;
  // Set start and goal pose
  pair<int, int> robotPose = make_pair(2, 0);
  pair<int, int> goalPose = make_pair(5, 5);
  cout << "Do you manually enter the world or run a demo? "
       << "\n1. Manually create map: m\n2. Run demo: d\n";
  char demo;
  cin >> demo;
  while (demo != 'm' && demo != 'd') {
    cout << "Please enter valid input." << endl;
    cout << "Do you manually enter the world or run a demo? "
         << "\n1. Manually create map: m\n2. Run demo: d\n";
    cin >> demo;
  }
  if (demo == 'm') {
    // Ask user for the world, robot & goal pose input
    cout << "Enter the number of width and length of the world: ";
    int length, width;
    cin >> width >> length;
    cout << "Enter the world elements row wise from left to right"
         << " and column wise top to bottom: " << endl;
    vector<int> x;
    int value;
    for (int i = 0; i < length; i++) {
      for (int j = 0; j < width; j++) {
        cin >> value;
        x.push_back(value);
      }
      // Store the elements in world
      worldState.push_back(x);
    }
    cout << "Enter the robot pose: ";
    cin >> robotPose.first >> robotPose.second;
    cout << "Enter the goal pose: ";
    cin >> goalPose.first >> goalPose.second;
  } else {
    worldState = {6, vector<int>(6, 0)};
    // Adding obstacles to map
    worldState[0][2] = 1;
    worldState[1][2] = 1;
    worldState[2][4] = 1;
    worldState[3][4] = 1;
    worldState[4][2] = 1;
    worldState[4][3] = 1;
    worldState[4][4] = 1;
  }
  vector<pair<int, int> > path;
  char planner;
  // Ask user to choose the planner
  cout << "Select your planner:\n1. Random: r\n2. Optimal: o\n";
  cin >> planner;
  // Check if user entered valid input for the planner
  while (planner != 'r' && planner != 'o') {
    cout << "Please enter valid input: " << endl;
    cout << "Select your planner:\n1. Random: r\n2. Optimal: o";
    cin >> planner;
  }
  // If the planner is random
  if (planner == 'r') {
    RandomPlanner R;
    if (demo == 'm') {
      cout << "Enter the maximum number of steps: ";
      cin >> R.maxStepNumber;
    }
    path = R.search(worldState, robotPose, goalPose);
    if (path.size() != 0) {
      R.M.showMap(path);
    }
  } else {
    OptimalPlanner O;
    path = O.search(worldState, robotPose, goalPose);
  }
  return 0;
}
