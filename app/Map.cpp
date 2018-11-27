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

#include <gnuplot-iostream.h>
#include <iostream>
#include <map>
#include <vector>
#include <utility>
#include "Map.hpp"

using std::vector;
using std::pair;
using std::make_pair;

Map::Map() {
  // Initializing values to the attributes of Map class
  length = 10;
  width = 10;
}

Map::~Map() {
  // Destructor stub
}

void Map::showMap(vector<pair<int, int> > path) {
  vector<pair<int, int> > xy_pts;
  // Add obstacle points to xy_pts to display
  for (int i = 0; i < length; i++) {
    for (int j = 0; j < width; j++) {
      if (world[i][j] == 1) {
        xy_pts.push_back(make_pair(j, -i));
      }
    }
  }
  // Interchange x and y and negate old x-axis
  vector<pair<int, int> > start;
  vector<pair<int, int> > end;
  start.push_back(make_pair(startNode.second, -startNode.first));
  end.push_back(make_pair(endNode.second, -endNode.first));
  vector<pair<int, int> > disp;
  for (unsigned int i = 0; i < path.size(); i++) {
    disp.push_back(make_pair(path[i].second, -path[i].first));
  }
  Gnuplot gp;
  // Create a plot of length l(Y-axis) and width w(X-axis)
  gp << "set xrange [" << -0.25 << ":" << -0.5 + length << "]\nset yrange ["
     << 0.5 - width << ":" << 0.25
     << "]\n";
  gp << "set title \"Planner Output\"\n";
  gp << "set pointsize 1\n";
  gp << "set xlabel \"Width\"\n";
  gp << "set ylabel \"Length\"\n";
  gp << "unset tics\n";
  gp << "set key outside\n";
  gp << "plot '-' with points pointtype 7 title 'Obstacle' , "
     << "'-' with lines title 'Path' , "
     << "'-' with points pointtype 2 title 'Start node' , "
     << "'-' with points pointtype 5 title 'End node'" << std::endl;
  gp.send1d(xy_pts);
  gp.send1d(disp);
  gp.send1d(start);
  gp.send1d(end);
}

bool Map::verifyNodes(std::map<pair<int, int>, int> visitedNodes,
                      pair<int, int> node) {
  bool verify = false;
  // Verify if new node is inside the boundaries of map
  if (node.first >= 0 && node.first < width) {
    if (node.second >= 0 && node.second < length) {
      // Verify if new node is not inside the
      // obstacle space or visited before
      if (visitedNodes[node] == 0) {
        verify = true;
      }
    }
  }
  return verify;
}

pair<int, int> Map::action(int i, pair<int, int> currentNode) {
  pair<int, int> newNode;
  switch (i) {
    case 0:
      newNode = currentNode;
      break;
    case 1:
      newNode = make_pair(currentNode.first + 1, currentNode.second);
      break;
    case 2:
      newNode = make_pair(currentNode.first, currentNode.second + 1);
      break;
    case 3:
      newNode = make_pair(currentNode.first - 1, currentNode.second);
      break;
    case 4:
      newNode = make_pair(currentNode.first, currentNode.second - 1);
      break;
  }
  return newNode;
}
