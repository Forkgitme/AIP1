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
    return  [s, s, w, s, w, w, s, w]

def genericGraphSearch(problem, strategyQueue):
    " get the start state and put it in the queue "
    startState = (problem.getStartState(), [], 0)
    strategyQueue.push(startState)
    " make an empty list to put all visited states in it "
    visitedStates = []
    " while the queue is not empty, dequeue the item in the queue (concerning which strategy you use) "
    while not strategyQueue.isEmpty():
        " unpack the tuples in a state to use the variable individual --> example: can also be written as currentState = firstState[0] etc. "
        currentState, currentActions, currentCost = strategyQueue.pop()
        " if the current state is not yet visited, add the current state to the visitedStates list"
        if currentState not in visitedStates:
            visitedStates.append(currentState)
            " check if the current state is the goal state "
            if problem.isGoalState(currentState):
                return currentActions
            " get the successors of the current state, update the values and put them in the queue "
            for successorState, successorAction, successorCost in problem.getSuccessors(currentState):
                " creates a copy of the currentActions list "
                totalAction = list(currentActions)
                totalAction.append(successorAction)
                totalCost = currentCost + successorCost
                successorState = (successorState, totalAction, totalCost)
                strategyQueue.push(successorState)

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
    " dfs fringe is LIFO --> Stack "
    dfsStack = util.Stack()
    return genericGraphSearch(problem, dfsStack)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    " bfs fringe is FIFO --> Queue "
    bfsQueue = util.Queue()
    return genericGraphSearch(problem, bfsQueue)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    " Priority Queue! "
    "Set initial cost to 0"
    cost = 0
    "Define starting node of the problem"
    node = (problem.getStartState(), [], cost)
    "Define prority queue"
    openNodes = util.PriorityQueue()
    openNodes.push(node, cost)
    "Define list to hold already evaluated nodes"
    closedNodes = []
    "While the optimal path is not yet found"
    while not openNodes.isEmpty():
        "Retrieve the node that looks most promising"
        currentState, currentActions, currentCost = openNodes.pop()
        "If returned node is our goal we are finished"
        if problem.isGoalState(currentState):
            return currentActions
        if currentState not in closedNodes:
            "Add node to evaluated nodes"
            closedNodes.append(currentState)
            for successorState, successorAction, successorCost in problem.getSuccessors(currentState):
                "Creates a copy of the currentActions list "
                totalAction = list(currentActions)
                totalAction.append(successorAction)
                "Calculate new cost"
                totalCost = currentCost + successorCost
                "Make a node with updated information"
                n = (successorState, totalAction, totalCost)
                "If the node was not yet evaluated add the node to the queue"
                openNodes.push(n, totalCost)
		
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    "Determine the heuristic of the start node"
    cost = heuristic(problem.getStartState(), problem)
    "Define initial state of the problem"
    node = (problem.getStartState(), [], 0)
    "Create priority queue to determine the node to expand next"
    openNodes = util.PriorityQueue()
    openNodes.push(node, cost)
    "Make a list to track which nodes are already evaluated to prevent infinite looping"
    closedNodes = []
    "While shortest path is not yet found continue searching"
    while not openNodes.isEmpty():
        "Retrieve the node that is most promising from the open nodes"
        currentState, currentActions, currentCost = openNodes.pop()
        "If the evaluated state is our goal return the path"
        if problem.isGoalState(currentState):
            return currentActions
        "Mark node as evaluated"
        if currentState not in closedNodes:
            closedNodes.append(currentState)
            "Evaluate each of the child nodes"
            for successorState, successorAction, successorCost in problem.getSuccessors(currentState):
                "Count all actions to reach this state"
                totalAction = list(currentActions)
                totalAction.append(successorAction)
                "Evaluate the cost until now and add heuristics"
                g_cost = currentCost + successorCost
                totalCost = g_cost + heuristic(successorState, problem)
                "Define a node contaning the new information"
                n = (successorState, totalAction, g_cost)
                "Add node to be evaluated"
                openNodes.push(n, totalCost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
