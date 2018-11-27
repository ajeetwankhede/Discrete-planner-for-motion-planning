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

#include <math.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>
#include "Map.hpp"
#include "Output.hpp"
#include "OptimalPlanner.hpp"

using std::vector;
using std::pair;
using std::make_pair;
using std::cout;
using std::endl;

OptimalPlanner::OptimalPlanner() {
  // Initializing values to the attributes of Map class
  noOfNodes = 0;
  time = 0.0;
}

OptimalPlanner::~OptimalPlanner() {
  // Destructor stub
}

void OptimalPlanner::initMap() {
  // Create new key value pairs
  // Create new key value pair and initialize to zero if key not found
  cout << "Entered World is: " << endl;
  for (int i = 0; i < M.length; i++) {
    for (int j = 0; j < M.width; j++) {
      // Initialize parent cost to zero
      parentCost[make_pair(i, j)] = 0.0;
      // Initialize current cost to zero
      currentCost[make_pair(i, j)] = 0.0;
      cout << M.world[i][j] << "\t";
      // Check if key value pair in obstacle already created
      if (M.world[i][j] == 0) {
        // Initialize visited nodes to zero
        visitedNodes[make_pair(i, j)] = 0;
      } else {
        // Add obstacle space to visited nodes
        visitedNodes[make_pair(i, j)] = 1;
      }
    }
    cout << "\n";
  }
}

vector<pair<int, int> > OptimalPlanner::search(vector<vector<int> > worldState,
                                              pair<int, int> robotPose,
                                              pair<int, int> goalPose) {
  // Assign the vales to Map's attribute
  M.world = worldState;
  M.startNode = robotPose;
  M.endNode = goalPose;
  M.length = worldState[0].size();
  M.width = worldState.size();
  // Call the initMap to initialize the variables
  initMap();
  // Start the timer
  clock_t start = clock();
  // Verify the start and goal pose to be within map boundaries
  // and outside obstacle space
  if (!M.verifyNodes(visitedNodes, robotPose)) {
    cout << "Robot pose (" << robotPose.first << ", " << robotPose.second
        << ") input is wrong." << endl;
    return path;
  }
  if (!M.verifyNodes(visitedNodes, goalPose)) {
    cout << "Goal pose (" << goalPose.first << ", " << goalPose.second
        << ") input is wrong." << endl;
    return path;
  }
  pair<int, int> newNode;
  pair<int, int> currentNode = robotPose;
  vector<pair<double, pair<int, int> > > queue;
  // Add the cost and start node to the queue
  queue.push_back(make_pair(0.0, robotPose));
  bool search = false;
  cout << "Search started..." << endl;
  while (search == false) {
    // Sort the queue to find the node with minimum cost
    sort(queue.begin(), queue.end());
    // Use the minimum cost node to expand next
    currentNode = queue[0].second;
    // Move the robot in 8 connected space
    int i = 1;
    while (i <= 4) {
      // Decide which action to take
      newNode = M.action(i, currentNode);
      // Verify if new node is within the map boundaries and not visited
      if (M.verifyNodes(visitedNodes, newNode)) {
        // Add the new node to the visited nodes
        visitedNodes[newNode] = 1;
        double cost = 1.0;
        double dist = pow(newNode.first - goalPose.first, 2)
            + pow(newNode.second - goalPose.second, 2);
        // Calculate the cost of the new node
        cost += currentCost[currentNode] + sqrt(dist);
        // Assign cost of the new node
        currentCost[newNode] = cost;
        // Add the cost and new node to the queue
        queue.push_back(make_pair(cost, newNode));
        // Add the parent of new node
        parentNode[newNode] = currentNode;
        noOfNodes++;
        // Check if goal is reached
        if (newNode.first == goalPose.first
            && newNode.second == goalPose.second) {
          search = true;
          cout << "The search has been completed." << endl;
          break;
        }
      }
      i++;
    }
    // Remove the minimum cost node after saved as current node
    queue.erase(queue.begin());
    // Check if queue is empty
    if (queue.size() == 0) {
      cout << "The search queue is empty." << endl;
      break;
    }
  }
  cout << "Search completed." << endl;
  if (search == true) {
    cout << "Number of nodes explored: " << noOfNodes << endl;
    cout << "Path found." << endl;
    // Find the optimal path
    search = false;
    pair<int, int> parent = goalPose;
    // Add the end node to the path
    path.push_back(parent);
    while (search == false) {
      // Find the parent node
      parent = parentNode[parent];
      // Add the parent node to the path
      path.push_back(parent);
      // Check if start node reached
      if (parent.first == robotPose.first
          && parent.second == robotPose.second) {
        search = true;
      }
    }
    // Reverse the trajectory vector
    std::reverse(path.begin(), path.end());
    cout << "Optimal path found." << endl;
    Output S;
    // Show the trajectory coordinates
    S.showOutput(path);
    // Save the path to a text file
    S.writeTextFile(path);
  } else {
    cout << "The search could not find an optimal path." << endl;
  }
  // Stop the timer
  clock_t stop = clock();
  // Calculate the time difference
  time = static_cast<double>((stop - start) * 1000.0 / CLOCKS_PER_SEC);
  cout << "Time elapsed: " << time << " ms." << endl;
  return path;
}
