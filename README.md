# RoboticsAlgorithms
Collection of Robotic Algorithms 
from AtsushiSakai/PythonRobotics onlytailei/CppRobotics and AtsushiSakai/MATLABRobotics

# Table Of Contents
   * [What is this?](#what-is-this)
   * [Requirements](#requirements)
   * [Documentation](#documentation)
   * [How to use](#how-to-use)
   * [Localization](#localization)
      * [Extended Kalman Filter localization](#extended-kalman-filter-localization)
   * [SLAM](#slam)
      * [Iterative Closest Point (ICP) Matching](#iterative-closest-point-icp-matching)
   * [Path Planning](#path-planning)
      * [Grid based search](#grid-based-search)
         * [Dijkstra algorithm](#dijkstra-algorithm)
         * [A* algorithm](#a-algorithm)
   * [Path Tracking](#path-tracking)
      * [PID](#pid)
      
# What is this?

This is a code collection of robotics algorithms in Python C++ and Matlab, especially for autonomous navigation.

Features:

1. Easy to read for understanding each algorithm's basic idea.

2. Widely used and practical algorithms are selected.

3. Minimum dependency.

See this paper for more details:




# Requirements

PYTHON

- Python 3.7.x (2.7 is not supported)

- numpy

- scipy

- matplotlib

- pandas

- [cvxpy](https://www.cvxpy.org/index.html) 

C++

- cmake

- opencv 3.3

- Eigen 3

Matlab



# Documentation



# How to use



# Localization

## Extended Kalman Filter localization

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/extended_kalman_filter/animation.gif" width="640" alt="EKF pic">

Documentation: [Notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter_localization.ipynb)



# SLAM

Simultaneous Localization and Mapping(SLAM) examples

## Iterative Closest Point (ICP) Matching

This is a 2D ICP matching example with singular value decomposition.

It can calculate a rotation matrix and a translation vector between points to points.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/iterative_closest_point/animation.gif)

Ref:

- [Introduction to Mobile Robotics: Iterative Closest Point Algorithm](https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf)

## Batch Bundle Adjustment


# Path Planning


## Grid based search

### Dijkstra algorithm

This is a 2D grid based shortest path planning with Dijkstra's algorithm.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/Dijkstra/animation.gif)

In the animation, cyan points are searched nodes.

### A\* algorithm

This is a 2D grid based shortest path planning with A star algorithm.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AStar/animation.gif)

In the animation, cyan points are searched nodes.

Its heuristic is 2D Euclid distance.

# Path Tracking

## PID
