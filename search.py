# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

def to_bin(decimal_number, length):
    binary_string = bin(decimal_number)[2:]  # Convert decimal to binary, remove '0b' prefix
    padded_binary_string = binary_string.zfill(length)  # Pad with zeros to reach the specified length

    return padded_binary_string

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from util import Stack

    stack = Stack()
    stack.push((problem.getStartState(), []))
    path_visited = []
    path = None
    while not stack.isEmpty():
        curr_state, curr_path = stack.pop()
        if problem.isGoalState(curr_state):
            path = curr_path
            break
        # Skip redundant State
        if curr_state in path_visited:
            continue
        successors = problem.getSuccessors(curr_state)
        for successor in successors:
            stack.push((successor[0], curr_path + [successor[1]]))
        path_visited.append(curr_state)

    return path
    # util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    from util import Queue

    queue = Queue()
    queue.push((problem.getStartState(), []))
    path = None
    from searchAgents import CornersProblem
    # Corners Problem
    if isinstance(problem, CornersProblem):
        # Create dict for state corner_labels
        corner_labels = {to_bin(key, len(problem.corners)): [] for key in range(0, 16)}
        while not queue.isEmpty():
            curr_state, curr_path = queue.pop()
            if problem.isGoalState(curr_state):
                path = curr_path
                break
            corner_state = curr_state['corner']
            # Skip visited position in the same corner state label
            if curr_state['pos'] in corner_labels[corner_state]:
                continue
            successors = problem.getSuccessors(curr_state)
            for successor in successors:
                queue.push((successor[0], curr_path + [successor[1]]))
            corner_labels[corner_state].append(curr_state['pos'])

    # Position Problem
    else:
        path_visited = []
        while not queue.isEmpty():
            curr_state, curr_path = queue.pop()
            if problem.isGoalState(curr_state):
                path = curr_path
                break
            if curr_state in path_visited:
                continue
            successors = problem.getSuccessors(curr_state)
            for successor in successors:
                queue.push((successor[0], curr_path + [successor[1]]))
            path_visited.append(curr_state)

    return path

    # util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    from util import PriorityQueue
    priority_queue = PriorityQueue()
    priority_queue.push((problem.getStartState(), [], 0), 0)
    path_visited = []
    path = None
    while not priority_queue.isEmpty():
        curr_state, curr_path, curr_g = priority_queue.pop()
        if problem.isGoalState(curr_state):
            path = curr_path
            break
        if curr_state in path_visited:
            continue
        successors = problem.getSuccessors(curr_state)
        for successor in successors:
            new_path = curr_path + [successor[1]]
            new_g = curr_g + successor[2]
            priority_queue.push((successor[0], new_path, new_g), new_g)
        path_visited.append(curr_state)

    return path

    # util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    priority_queue = PriorityQueue()
    priority_queue.push((problem.getStartState(), [], 0), 0.0)
    path = None
    # Queue Ele: (State, path, cost)
    # Priority: f value
    # push(ele, priority)

    # Corners Problem
    from searchAgents import CornersProblem
    if isinstance(problem, CornersProblem):
        corner_labels = {to_bin(key, len(problem.corners)): [] for key in range(0, 16)}
        while not priority_queue.isEmpty():
            curr_state, curr_path, curr_g = priority_queue.pop()

            if problem.isGoalState(curr_state):
                path = curr_path
                break
            # Estimate f value
            h_value = heuristic(curr_state, problem)
            g_value = curr_g
            f_value = h_value + g_value

            corner_state = curr_state['corner']
            flag = False
            # Check each space
            for visited_node in corner_labels[corner_state]:
                # Compare visited node in same space
                if curr_state['pos'] == visited_node[0]['pos']:
                    if f_value < visited_node[1]:
                        corner_labels[corner_state].remove(visited_node)
                    else:
                        flag = True
                    break

            if flag:
                continue
            corner_labels[corner_state].append((curr_state, f_value))
            successors = problem.getSuccessors(curr_state)
            for successor in successors:
                h_value = heuristic(successor[0], problem)
                g_value = curr_g + successor[2]
                f_value = h_value + g_value
                new_path = curr_path + [successor[1]]
                priority_queue.push((successor[0], new_path, g_value), f_value)

    # Position Problem
    else:
        path_visited = []
        while not priority_queue.isEmpty():
            curr_state, curr_path, curr_g = priority_queue.pop()
            if problem.isGoalState(curr_state):
                path = curr_path
                break
            # Estimate f value
            h_value = heuristic(curr_state, problem)
            g_value = curr_g
            f_value = h_value + g_value
            flag = False
            for visited_node in path_visited:
                if curr_state == visited_node[0]:
                    if f_value < visited_node[1]:
                        path_visited.remove(visited_node)
                    else:
                        flag = True
                    break

            if flag:
                continue

            path_visited.append((curr_state, f_value))
            successors = problem.getSuccessors(curr_state)
            for successor in successors:
                h_value = heuristic(successor[0], problem)
                g_value = curr_g + successor[2]
                f_value = h_value + g_value
                new_path = curr_path + [successor[1]]
                priority_queue.push((successor[0], new_path, g_value), f_value)

    return path
    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

