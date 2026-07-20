
# SUDOS - Smart Urban Delivery Optimization System

SUDOS is a C-based project that simulates how delivery routes are planned in a city. It models the city as a graph and applies different algorithms to find efficient delivery routes for multiple agents.

The main idea is simple:
Given a set of delivery locations, how can we plan routes that minimize travel distance or time?

This project compares multiple algorithmic approaches to understand the trade-off between **speed and optimality**.


## Project Demo & Presentation

*  **Video Presentation & Code Demo**
  [https://youtu.be/UxvUcASbyY8?si=k9QF8z_6SqrkA5u2](https://youtu.be/UxvUcASbyY8?si=k9QF8z_6SqrkA5u2)

* **Project PPT**
  [https://docs.google.com/presentation/d/1wTB1z1gZAgw_M_Gt3TrazlxXvqLPwWiI/edit?slide=id.p1#slide=id.p1](https://docs.google.com/presentation/d/1wTB1z1gZAgw_M_Gt3TrazlxXvqLPwWiI/edit?slide=id.p1#slide=id.p1)


##  Features

* Models a city as a **weighted graph**

* Supports **multiple delivery agents**

* Uses different algorithms to compute routes:

  * Dijkstra’s Algorithm (shortest paths)
  * Greedy Nearest Neighbour
  * Dynamic Programming (TSP)
  * Nearest Insertion Heuristic

* Compares:

  * Total route cost
  * Runtime performance


##  Algorithms Used

### 1. Dijkstra’s Algorithm

Used to precompute shortest paths between all locations.
This helps all other algorithms work faster.

### 2. Greedy Nearest Neighbour

A fast and simple approach where the agent always goes to the nearest unvisited location.
 Very fast
 Not always optimal

### 3. DP-based TSP (Held-Karp)

Finds the **optimal route** using dynamic programming.
 Best possible solution
 Only works for small inputs (n ≤ 15)

### 4. Nearest Insertion Heuristic

Builds a route step by step by inserting the closest node in the best position.
 Good balance between speed and accuracy
 Works well for larger inputs


## How It Works

1. Build the city graph using an adjacency matrix
2. Compute shortest paths between all nodes
3. Assign delivery locations to agents
4. Run different routing algorithms
5. Compare results (cost + runtime)


##  Sample Output

The program prints:

* Route for each agent
* Total distance (cost)
* Comparison between Greedy, DP, and Heuristic methods


## Technologies Used

* Language: **C**
* Libraries:

  * `stdio.h`
  * `stdlib.h`
  * `string.h`
  * `limits.h`
  * `time.h`


## Key Insights

* Greedy is fastest but not always accurate
* DP gives the best result but is not scalable
* Nearest Insertion provides a good balance
* No single algorithm is best for all cases

---

##  Future Improvements

* Add real-time route updates
* Support time-window constraints (TWVRP)
* Use metaheuristics like Genetic Algorithms or Ant Colony Optimization
* Improve delivery assignment using optimization techniques

