/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    RandomPlanner.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  RandomPlanner class definition of methods.
 */

#include <math.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <utility>
#include <random>
#include <map>
#include <Map.hpp>
#include <Output.hpp>
#include "RandomPlanner.hpp"

using std::vector;
using std::pair;
using std::make_pair;
using std::cout;
using std::cin;
using std::endl;

RandomPlanner::RandomPlanner() {
  // Initializing values to the attributes of Map class
  noOfNodes = 0;
  maxStepNumber = 100;
  time = 0.0;
}

RandomPlanner::~RandomPlanner() {
  // Destructor stub
}

void RandomPlanner::initMap() {
  // Create new key value pairs
  // Create new key value pair and initialize to zero if key not found
  cout << "Entered World is: " << endl;
  for (int i = 0; i < M.length; i++) {
    for (int j = 0; j < M.width; j++) {
      cout << M.world[i][j] << "\t";
      // Initialize visited nodes to zero
      visitedNodes[make_pair(i, j)] = 0;
      // Check if key value pair in obstacle already created
      if (M.world[i][j] == 0) {
        // Initialize obstacle space to zero
        obstacleSpace[make_pair(i, j)] = 0;
      } else
        // Add obstacle space to visited nodes
        obstacleSpace[make_pair(i, j)] = 1;
    }
    cout << "\n";
  }
}

vector<pair<int, int> > RandomPlanner::search(vector<vector<int> > worldState,
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
  auto start = std::chrono::steady_clock::now();
  // Verify the start and goal pose to be within map boundaries
  // and outside obstacle space
  if (!M.verifyNodes(obstacleSpace, robotPose)) {
    cout << "Robot pose (" << robotPose.first << ", " << robotPose.second
         << ") input is wrong." << endl;
    return path;
  }
  if (!M.verifyNodes(obstacleSpace, goalPose)) {
    cout << "Goal pose (" << goalPose.first << ", " << goalPose.second
         << ") input is wrong." << endl;
    return path;
  }
  pair<int, int> currentNode = robotPose, newNode;
  vector<pair<int, int> > queue;
  pair<int, int> lastNode;
  bool flag = false;
  vector<int> actionList { 1, 2, 3, 4 };
  queue.push_back(currentNode);
  std::random_device rd;
  std::mt19937 rng(rd());
  int steps = 0, action, index;
  path.push_back(currentNode);
  cout << "Search started..." << endl;
  while (noOfNodes < maxStepNumber && currentNode != goalPose) {
    // Check is action list is not empty
    if (actionList.size() != 0) {
      // Distribution in range [,]
      std::uniform_int_distribution<> dist(0, actionList.size() - 1);
      index = dist(rng);
      action = actionList[index];
      // Remove the selected action from the list
      actionList.erase(actionList.begin() + index);
    } else {
      action = 0;
    }
    newNode = M.action(action, currentNode);
    // Execute the previous valid node if no action is remaining to take
    // newNode == currentNode checking that action is zero second time
    if (actionList.size() == 0 && newNode == currentNode && flag == true) {
      newNode = lastNode;
      flag = false;
    }
    // Verify if newNode is inside the boundaries of map
    if (newNode.first >= 0 && newNode.first < M.width) {
      if (newNode.second >= 0 && newNode.second < M.length) {
        // Check if the newNode is outside obstacle space
        if (obstacleSpace[newNode] == 0) {
          // Check if the newNode is already visited
          if (visitedNodes[newNode] == 1) {
            lastNode = newNode;
            flag = true;
            // Check if all actions are executed
            if (actionList.size() == 0) {
              // Clear the visited node flag
              visitedNodes[newNode] = 0;
            }
          }
          // If the newNode is not visited then perform following actions
          if (visitedNodes[newNode] == 0) {
            // Add the new node to the visited nodes
            visitedNodes[newNode] = 1;
            // Add the new node to path
            path.push_back(newNode);
            // Add the new node to the queue
            queue.push_back(newNode);
            noOfNodes++;
            steps++;
            // Clear visited node from memory
            if (steps > (int) sqrt(maxStepNumber)) {
              visitedNodes[queue[0]] = 0;
              steps = 0;
              // Remove the first node
              queue.erase(queue.begin());
            }
            // Use the new node to expand next
            currentNode = newNode;
            // Set the action list
            actionList = {1, 2, 3, 4};
          }
        }
      }
    }
  }
  cout << "Search completed." << endl;
  cout << "Number of nodes explored: " << noOfNodes << endl;
  // Find the path
  if (currentNode == goalPose) {
    cout << "Path found." << endl;
    Output S;
    // Show the trajectory coordinates
    S.showOutput(path);
    // Save the path to a text file
    S.writeTextFile(path);
  } else {
    cout << "The search could not find a path." << endl;
    path = {};
  }
  // Stop the timer
  auto end = std::chrono::steady_clock::now();
  // Calculate the time difference
  std::chrono::duration<double> elapsed = end - start;
  // Store the number of seconds
  time = elapsed.count();
  cout << "Time elapsed: " << time << " seconds." << endl;
  return path;
}
