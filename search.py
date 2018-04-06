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
  def __init__(self, state, action, cost, parent):
    self.state = state
    self.action = action
    self.cost = cost
    self.parent = parent
    
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


def graphSearchDfs(problem):
  """
  Graph Search DFS implementation
  """
  frontier = util.Stack()
  node = Node(problem.getStartState(), None, None, None)
  frontier.push(node)
  explored = []
  while True:
    if frontier.isEmpty():
      return "ERROR in graphSearchDfs method"
      sys.exit(1)
    node = frontier.pop()
    
    # print "Exploring node: ", node
    # raw_input("Press Enter to continue...")
    if(problem.isGoalState(node.state)):
      sol = solution(node)
      return sol
    
    successors = problem.getSuccessors(node.state)
    #print "new successors:", successors
    for successor in successors:
      newNode = Node(successor[0], successor[1], successor[2], node)
      if (newNode.state not in explored) and (newNode not in frontier.list) :
        frontier.push(newNode)
        # print "New node pushed: ", newNode
      #else:
        # print "node not pushed: ", newNode
        
    explored.append(node.state)
    #print "Explored:" , explored

def graphSearch(problem, searchType = "dfs"):
  """
  Graph Search  implementation
  serachType is string define the search type.
  searchType options are:
     "dfs" for deep first search,
     "bfs" for bread first search

  """
  print "Running graphSearch function with search type: ", bfs
  if(searchType is "bfs"): frontier = util.Queue()
  else: frontier = util.Stack()
  node = Node(problem.getStartState(), None, None, None)
  frontier.push(node)
  explored = []
  while True:
    if frontier.isEmpty():
      return "ERROR in graphSearchBfs method"
      sys.exit(1)
    node = frontier.pop()
    if(searchType is "dfs"):
      if(problem.isGoalState(node.state)):
        sol = solution(node)
        return sol

    # print "Exploring node: ", node
    # raw_input("Press Enter to continue...")
    
    successors = problem.getSuccessors(node.state)
    # print "new successors:", successors
    for successor in successors:
      newNode = Node(successor[0], successor[1], successor[2], node)
      if (newNode.state not in explored) and (newNode not in frontier.list) :
        if(searchType is "bfs"):
          if(problem.isGoalState(newNode.state)):
            sol = solution(newNode)
            return sol
        frontier.push(newNode)
      #   print "New node pushed: ", newNode
      # else:
      #   print "node not pushed: ", newNode
    
    explored.append(node.state)
    # print "Explored:" , explored
      
      

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
  return graphSearch(problem, "dfs")

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"
  return graphSearch(problem, "bfs")
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
