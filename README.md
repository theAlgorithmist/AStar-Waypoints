# TypeScript A* for Waypoints Library

This is a TypeScript implementation of the A* algorithm for pathfinding in the plane.  A two-dimensional space is defined as a collection of waypoints, where a connection between waypoints A and B means that it is possible to travel from A to B. The A* algorithm computes a path as a sequence of waypoints that allows travel from the starting point to the destination at minimum cost.

This implementation is ideal for those wishing to learn more about the algorithm directly from a code base.  The current implementation uses Euclidean distance between waypoints to compute cost.  So, the minimum-distance path is returned and there is an underlying presumption that all paths between waypoints are along a straight line.  This condition can be relaxed and more effective cost heuristics could be substituted.  This effort is highly encouraged.  

In fact, it is possible to make the code reactive in the sense that new information affecting the cost of travel between two waypoints can be input to the pathfinder, and the best-possible path can be recomputed.  If you have ever used Google Maps or Waze to navigate to a destination, you have probably seen this in action :)

NOTE:  This repo has been deprecated - this code is now [updated and maintained as part of the AMYR library](https://github.com/theAlgorithmist/AMYR).

Author:  Jim Armstrong - [The Algorithmist]

@algorithmist

theAlgorithmist [at] gmail [dot] com

Typescript: 4.4.4

Jest: 27.0.7

Version: 1.0

## Installation

Installation involves all the usual suspects

  - npm installed globally
  - Clone the repository
  - npm install
  - get coffee (this is the most important step)


### Building and running the tests

1. npm t (it really should not be this easy, but it is)

2. Standalone compilation only (npm build)

Specs (_hull.spec.ts_) reside in the ___tests___ folder.


### Notes

This code is a fairly 'textbook' implementation of the A* algorithm for waypoints.  Many descriptions of the algorithm suggest a priority queue to store computed paths and costs.  This implementation uses a min-heap.  The necessary data structures (graph and min-heap) are optimized for direct use in the A* computations.


### Reference

For late bedtime reading,

[Path Planning, Waypoints, and Search](https://www.cs.utexas.edu/users/fussell/courses/cs378/lectures/cs378-20.pdf)


License
----

Apache 2.0

**Free Software? Yeah, Homey plays that**

[//]: # (kudos http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

[The Algorithmist]: <https://www.linkedin.com/in/jimarmstrong/>

