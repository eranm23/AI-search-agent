# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           
class Node:
  def __init__(self, state, action, cost, parent, problem=None, heuristic=None):
    self.state = state
    self.action = action
    self.cost = cost
    self.parent = parent
    self.problem = problem
    self.heuristic = heuristic
    
  def __str__(self):
    return str(self.state) + ", " + str(self.action) + ", " + str(self.cost)


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]




debug = False
def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 74].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  frontier = util.Stack()
  node = Node(problem.getStartState(), None, 0, None)
  frontier.push(node)
  explored = []
  while True:
    if frontier.isEmpty():
      return "ERROR in depthFirstSearch function"
      sys.exit(1)
    node = frontier.pop()

    if(debug): print "Exploring node: ", node
    if(debug): raw_input("Press Enter to continue...")
    
    successors = problem.getSuccessors(node.state)
    if(debug): print "new successors:", successors
    for successor in successors:
      childNode = Node(successor[0], successor[1], node.cost + successor[2], node)
      if (childNode.state not in explored) and (childNode not in frontier.list) :
        if(problem.isGoalState(childNode.state)):
          sol = solution(childNode)
          return sol
        frontier.push(childNode)
        if(debug): print "Child node pushed: ", childNode
      else:
        if(debug): print "Child node not pushed: ", childNode
    
    explored.append(node.state)
    if(debug): print "Explored:" , explored

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"

  frontier = util.Queue()
  node = Node(problem.getStartState(), None, 0, None)
  frontier.push(node)
  explored = []
  while True:
    if frontier.isEmpty():
      return "ERROR in breadthFirstSearch function"
      sys.exit(1)
    node = frontier.pop()

    if(debug): print "Exploring node: ", node
    if(debug): raw_input("Press Enter to continue...")
    
    successors = problem.getSuccessors(node.state)
    if(debug): print "new successors:", successors
    for successor in successors:
      childNode = Node(successor[0], successor[1], node.cost + successor[2], node)
      if (childNode.state not in explored) and (childNode not in frontier.list) :
        if(problem.isGoalState(childNode.state)):
          sol = solution(childNode)
          return sol
        frontier.push(childNode)
        if(debug): print "Child node pushed: ", childNode
      else:
        if(debug): print "Child node not pushed: ", childNode
    
    explored.append(node.state)
    if(debug): print "Explored:" , explored
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  frontier = util.PriorityQueueWithFunction(calcPriority)
  node = Node(problem.getStartState(), None, 0, None)
  frontier.push(node)
  explored = []
  while True:
    if frontier.isEmpty():
      return "ERROR in uniformCostSearch function"
      sys.exit(1)
    node = frontier.pop()
    if(problem.isGoalState(node.state)):
      sol = solution(node)
      return sol

    if(debug): print "Exploring node: ", node
    if(debug): raw_input("Press Enter to continue...")
    
    successors = problem.getSuccessors(node.state)
    if(debug): print "new successors:", successors
    for successor in successors:
      childNode = Node(successor[0], successor[1], node.cost + successor[2], node)
      if (childNode.state not in explored) and (childNode not in frontier.heap) :
        frontier.push(childNode)
        if(debug): print "Child node pushed: ", childNode
      else:
        if shouldAdd(childNode, frontier):
          frontier.push(childNode)
          if(debug): print "Chils node with less cost and need to therefore pushed: ", childNode
        else:
          if(debug): print "Child node not pushed: ", childNode
    
    explored.append(node.state)
    if(debug): print "Explored:" , explored

def solution(node):
  """
  returns an actions list represent a solution from a gole state node.
  """
  actions = []
  while node != None and node.action != None:
    actions.append(node.action)
    node = node.parent
  actions.reverse()  
  return actions


def shouldAdd(node, frontier):
  """
  Use only for UCS (uniform cost search) where frontier is priority queue.
  Returns true if frontier contain node with same state and with greater cost,
  otherwise returns false.
  """
  lst = frontier.heap
  for tpl in lst:
    if tpl[1].state == node.state and tpl[1].cost > node.cost:
      return True
  return False

def calcPriority(node):
  "Priority function for PriorityQueueWithFunction class usage"
  return node.cost

def calcPriorityH(node):
  "Priority function for PriorityQueueWithFunction class usage"
  return node.cost + node.heuristic(node.state, node.problem)




def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."

  frontier = util.PriorityQueueWithFunction(calcPriorityH)
  node = Node(problem.getStartState(), None, 0, None, problem, heuristic)
  frontier.push(node)
  explored = []
  while True:
    if frontier.isEmpty():
      return "ERROR in uniformCostSearch function"
      sys.exit(1)
    node = frontier.pop()
    if(problem.isGoalState(node.state)):
      sol = solution(node)
      return sol

    if(debug): print "Exploring node: ", node
    if(debug): raw_input("Press Enter to continue...")
    
    successors = problem.getSuccessors(node.state)
    if(debug): print "new successors:", successors
    for successor in successors:
      childNode = Node(successor[0], successor[1], node.cost + successor[2], node, problem, heuristic)
      if (childNode.state not in explored) and (childNode not in frontier.heap) :
        frontier.push(childNode)
        if(debug): print "Child node pushed: ", childNode
      else:
        if shouldAdd(childNode, frontier):
          frontier.push(childNode)
          if(debug): print "Chils node with less cost and need to therefore pushed: ", childNode
        else:
          if(debug): print "Child node not pushed: ", childNode
    
    explored.append(node.state)
    if(debug): print "Explored:" , explored
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
