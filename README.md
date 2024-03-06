# This is a repository for the project of State Space Search of the course Introduction to Atificial Inteligence at Ho Chi Minh Universiy of Technology (Bach Khoa HCM University).

This is a project to reproduce Pacman Game which already had built-in API for graphic and game. My contribution is to build optimal search algorithms to help Pacmann complete his game (search.py, searchAgent.py). <br>
Some of the algorithms used to optimize search are *Depth First Search, Breadth First Search, Uniform-cost Search, A-star Search.*<br>

Typing at the command line to execute the program with provided testcase: 
```
python autograder.py
```
You could play the game by typing at command line:
```
python pacman.py
```
These are the command line to excute solution solved in this project:<br>
# P1: Finding a Fixed Food Dot using Depth First Search<br>
To run the solution, typing at the command line:
```
python pacman.py -l tinyMaze -p SearchAgent
python pacman.py -l mediumMaze -p SearchAgent
python pacman.py -l bigMaze -z .5 -p SearchAgent
```

# P2: Breadth First Search
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5
python eightpuzzle.py
```
# P3: Varying the Cost Function
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
```
# P4: A* search
```
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
```
# P5: Finding All the Corners
```
python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
```
# P6: Corners Problem: Heuristic
```
python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5
```
# P7: Eating All The Dots
```
python pacman.py -l testSearch -p AStarFoodSearchAgent
python pacman.py -l trickySearch -p AStarFoodSearchAgent
```
# P8: Suboptimal Search
```
python pacman.py -l bigSearch -p ClosestDotSearchAgent -z .5
```
