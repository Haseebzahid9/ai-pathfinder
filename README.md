# AI Pathfinder --- Uninformed Search Visualizer

Assignment 1 · Question 7\
AI2002 Artificial Intelligence · Spring 2026

Haseeb Zahid (23F0644)\
Adan Sajid (23F-0691)

------------------------------------------------------------------------

## Project Description

This project is a step-by-step animated visualizer for six uninformed
(blind) search algorithms. The program navigates a 10 × 10 grid from
Start (1,1) to Goal (8,8) while avoiding static wall obstacles.

The purpose of this project is to demonstrate how different uninformed
search algorithms explore the grid and find a path to the goal.

------------------------------------------------------------------------

## Implemented Algorithms

1.  Breadth-First Search (BFS)\
2.  Depth-First Search (DFS)\
3.  Uniform Cost Search (UCS)\
4.  Depth-Limited Search (DLS)\
5.  Iterative Deepening Depth-First Search (IDDFS)\
6.  Bidirectional Search

Each algorithm displays: - Animated grid visualization\
- Nodes explored\
- Frontier size\
- Path length\
- Step count\
- Final path printed in the console

------------------------------------------------------------------------

## Movement Order

1.  Up (-1, 0)\
2.  Right (0, +1)\
3.  Down (+1, 0)\
4.  Down-Right (+1, +1)\
5.  Left (0, -1)\
6.  Top-Left (-1, -1)

Top-Right and Bottom-Left diagonals are not allowed.

------------------------------------------------------------------------

## Grid Configuration

Grid size: 10 × 10\
Start: (1, 1)\
Goal: (8, 8)

Static wall positions:

(2,3), (3,3), (4,3), (4,4), (4,5),\
(6,5), (6,6), (6,7), (5,7), (3,7)

------------------------------------------------------------------------

## GUI Color Meaning

Blue cell represents the Start node.\
Green cell represents the Goal node.\
Dark grey cells represent static walls.\
Cyan cells represent the frontier.\
Grey cells represent explored nodes.\
Amber cells represent the final path.\
Purple border shows the current node being expanded.

------------------------------------------------------------------------

## Requirements

Python 3.8 or higher\
matplotlib library

Install dependency:

pip install matplotlib

------------------------------------------------------------------------

## How to Run

1.  Open terminal in the project folder.\
2.  Run:

python ai_pathfinder.py

3.  Choose an algorithm from the menu.\
4.  The animation window will open automatically.

------------------------------------------------------------------------

## Project Structure

ai-pathfinder/\
ai_pathfinder.py\
README.md\
screenshots/

------------------------------------------------------------------------

## Educational Objective

This project demonstrates: - Differences between uninformed search
strategies\
- Completeness and optimality\
- Effect of search order\
- Visualization of frontier expansion\
- Performance comparison

------------------------------------------------------------------------

## License

This project was developed as coursework for AI2002 Artificial
Intelligence at FAST-NUCES, Spring 2026.
