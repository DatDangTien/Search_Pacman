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

    start_state = problem.getStartState()

    stack = Stack()
    visited = []
    path = None

    stack.push((start_state, []))

    while not stack.isEmpty():
        current_state = stack.pop()

        if problem.isGoalState(current_state[0]):
            path = current_state[1]
            break

        if current_state[0] in visited:
            continue

        successors = problem.getSuccessors(current_state[0])

        for successor in successors:
            stack.push((successor[0], current_state[1] + [successor[1]]))

        visited.append(current_state[0])

    return path
    # util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    from util import Queue

    start_state = problem.getStartState()

    queue = Queue()
    # visited = {to_bin(key, 4): [] for key in range(0,16)}
    # print(visited.keys())
    path = None
    queue.push((start_state, []))

    from searchAgents import CornersProblem
    if isinstance(problem, CornersProblem):
        visited = {to_bin(key, len(problem.corners)): [] for key in range(0, 16)}
        while not queue.isEmpty():
            current_state = queue.pop()
            # print(current_state)
            # import time
            # time.sleep(0.1)
            if problem.isGoalState(current_state[0]):
                path = current_state[1]
                # print("here")
                # print(path)
                break

            corner_state = current_state[0]['corner_state']
            if current_state[0]['position'] in visited[corner_state]:
                continue

            # print('here1')
            successors = problem.getSuccessors(current_state[0])
            # print(successors)
            for successor in successors:
                queue.push((successor[0], current_state[1] + [successor[1]]))


            # print('here3')
            visited[corner_state].append(current_state[0]['position'])
            # problem.visited.append(current_state[0])
            # print(queue.list)
    else:
        visited = []
        while not queue.isEmpty():
            current_state = queue.pop()
            # print(current_state)
            # import time
            # time.sleep(1)
            if problem.isGoalState(current_state[0]):
                path = current_state[1]
                break

            if current_state[0] in visited:
                continue

            successors = problem.getSuccessors(current_state[0])

            for successor in successors:
                queue.push((successor[0], current_state[1] + [successor[1]]))

            visited.append(current_state[0])

    print(path)
    return path

    # util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    from util import PriorityQueue

    start_state = problem.getStartState()

    priority_queue = PriorityQueue()
    visited = []
    path = None

    priority_queue.push((start_state, [], 0), 0)

    while not priority_queue.isEmpty():
        current_state = priority_queue.pop()
        # print(current_state)
        # import time
        # time.sleep(1)

        if problem.isGoalState(current_state[0]):
            path = current_state[1]
            break

        if current_state[0] in visited:
            continue

        successors = problem.getSuccessors(current_state[0])

        for successor in successors:
            # print(successor)

            new_path = current_state[1] + [successor[1]]
            new_cost = current_state[2] + successor[2]
            priority_queue.push((successor[0], new_path, new_cost), new_cost)

        visited.append(current_state[0])

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

    # from util import PriorityQueue
    #
    # start_state = problem.getStartState()
    #
    # priority_queue = PriorityQueue()
    # visited = []
    # path = None
    #
    # # Queue Ele: (State, path, cost)
    # # Priority: f func
    # # push(ele, priority)
    # priority_queue.push((start_state, [], 0), 0.0)
    #
    # while not priority_queue.isEmpty():
    #     # priority_queue.print()
    #     current_state = priority_queue.pop()
    #     # print(current_state)
    #     # import time
    #     # time.sleep(1)
    #
    #     if problem.isGoalState(current_state[0]):
    #         path = current_state[1]
    #         break
    #
    #     h_value = heuristic(current_state[0], problem)
    #     g_value = current_state[2]
    #     f_value = h_value + g_value
    #
    #     # print(current_state[0], f_value)
    #     # import time
    #     # time.sleep(1)
    #
    #     flag = False
    #     for visited_node in visited:
    #         if current_state[0] == visited_node[0]:
    #             if f_value < visited_node[1]:
    #                 visited.remove(visited_node)
    #             else:
    #                 flag = True
    #
    #             break
    #
    #     if flag:
    #         continue
    #
    #     visited.append((current_state[0], f_value))
    #
    #     successors = problem.getSuccessors(current_state[0])
    #
    #     for successor in successors:
    #         # print(successor)
    #         # import time
    #         # time.sleep(1)
    #         h_value = heuristic(successor[0], problem)
    #         g_value = current_state[2] + successor[2]
    #         f_value = h_value + g_value
    #
    #         new_path = current_state[1] + [successor[1]]
    #
    #         priority_queue.push((successor[0], new_path, g_value), f_value)
    #
    # return path

    from util import PriorityQueue

    start_state = problem.getStartState()

    priority_queue = PriorityQueue()
    path = None
    visited = {to_bin(key, len(problem.corners)): [] for key in range(0, 16)}

    # Queue Ele: (State, path, cost)
    # Priority: f func
    # push(ele, priority)
    priority_queue.push((start_state, [], 0), 0.0)

    while not priority_queue.isEmpty():
        # priority_queue.print()
        current_state = priority_queue.pop()
        # print(current_state)
        # import time
        # time.sleep(1)

        if problem.isGoalState(current_state[0]):
            path = current_state[1]
            break

        h_value = heuristic(current_state[0], problem)
        g_value = current_state[2]
        f_value = h_value + g_value

        # print(current_state[0], f_value)
        # import time
        # time.sleep(1)

        flag = False
        # Check each space?
        for visited_node in visited:
            # Compare visited node in same space?
            if current_state[0] == visited_node[0]:
                if f_value < visited_node[1]:
                    visited.remove(visited_node)
                else:
                    flag = True

                break

        if flag:
            continue

        visited.append((current_state[0], f_value))

        successors = problem.getSuccessors(current_state[0])

        for successor in successors:
            # print(successor)
            # import time
            # time.sleep(1)
            h_value = heuristic(successor[0], problem)
            g_value = current_state[2] + successor[2]
            f_value = h_value + g_value

            new_path = current_state[1] + [successor[1]]

            priority_queue.push((successor[0], new_path, g_value), f_value)

    return path

    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

