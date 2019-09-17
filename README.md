# Mobile-Robot-Path-Planner

This is a mobile robot path planner feature, which indicates the shortest path/lowest time a robot would take to reach the goal.

Robot has to go through certain intermediate points and pause for some time to do some task, failure to do so/omit them will invoke
penalty time.

The path planner leverages Dijkstra's single source shortest path algorithm for a graph and is implemented in C++.
