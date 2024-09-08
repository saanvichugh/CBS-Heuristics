import matplotlib.pyplot as plt
import networkx as nx
from copy import copy
import random
from queue import Queue
import time
import numpy as np

# CBS outline:
     # run aStar once with [] constraints
     # pass through calculated constraints + rereun aStar
     # run  findConflicts functions again
     # pass thorugh any new conflicts found
     # so on and so forth...

# def findPath(start, goal):

# random.seed(0)

G = nx.grid_2d_graph(4, 4)


def dist(a, b):
        (x1, y1) = a
        (x2, y2) = b
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

# collects gCosts in a dictionary within A* function below 

# calculates hCost (distance from current node and end node)
def hCost(currentNode, end):
        return dist(currentNode, end)

# calculates fCost (gCost + hCost)
def fCost(gCost, currentNode, end):
        fCost = gCost + hCost(currentNode, end)
        return fCost
# I got rid of the gCost = len(path) stuff for now!

def get_adjacent_nodes(currentNode):
    # possible movements: up, down, left, right
    movements = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    children = []
    for move in movements:
        adjacent_node = (currentNode[0] + move[0], currentNode[1] + move[1])

        if 0 <= adjacent_node[0] < 10 and 0 <= adjacent_node[1] < 10:
            children.append(adjacent_node)
    return children

#---------------------------------------------------------------------------------------------------------

# code to generate instances (start and goal locations for a given number of agents)

def genInstances(agents):

    starts = set()
    ends = set()
    for _ in range(agents):
        starts.add((random.randint(0, 9), random.randint(0, 9)))
        ends.add((random.randint(0, 9), random.randint(0, 9)))

    while len(starts) > len(ends):
         diff = len(starts) - len(ends)
         for _ in range(diff):
            ends.add((random.randint(0, 9), random.randint(0, 9)))

    while len(starts) < len(ends):
         diff = len(ends) - len(starts)
         for _ in range(diff):
            starts.add((random.randint(0, 9), random.randint(0, 9)))

    start_locations = list(starts)
    goal_locations = list(ends)

    return start_locations, goal_locations

# ---------------------------------------------------------------------------------------------------------

def plot_paths(paths):
    plt.figure(figsize=(9, 9))

    for path in paths:
            for coordinate in path:
               x, y = zip(*coordinate)  # Unpack the coordinate pairs into x and y lists
               plt.plot(x, y, marker='o', linestyle='-', label=f'Path {path.index(coordinate) + 1}')

    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Multiple Paths Plot')

    plt.legend()
    plt.show()

# -----------------------------------------------------------------------------------------------------------------
# CBS:

# make parent node 
# node is defined by path A and its constraints + path B and its constraints
# each node is a list

def CBS(startPoints, endPoints, heuristic):
     parentNode = []
     for startpoint in startPoints: 
          parentNode.append(aStar(startpoint, endPoints[startPoints.index(startpoint)], []))
          parentNode.append([])
    
     queue = Queue(maxsize = 0)

# all constraints are [] right now since no constraints at parent node
     def gen2Nodes(node):

        constraints = []
        for i in range(len(node)):
             if i % 2 != 0:
                  constraints.append(node[i])

        paths = []
        for i in range(len(node)):
             if i % 2 == 0:
                  paths.append(node[i])

          #  paths now contains all paths at this node
          # constraints now contains all paths at this node

        # the following line is for the original version of CBS (first conflict resolution)
        # constraint1 = findFirstConflict(paths) 

        # new line for all the different heuristics

        constraint1 = chooseConflict(paths, heuristic)

        if constraint1 is None:
            #  return paths
             print("final paths:", paths)
             return None, None, paths
        # if no conflicts, final paths found
        else: 


             if constraint1[0] == 'vertex':
               node1 = []
               node2 = []
               for index, path in enumerate(paths):
                    if index != constraint1[3] and index != constraint1[4]: 
                    # if not a conflicting path:
                        #  print(index)
                         node1.append(path)
                         node1.append(constraints[index])
                         node2.append(path) 
                         node2.append(constraints[index]) 
                    else:
                         if index == constraint1[3]:
                    # node1 applies conflict1 to the first conflicting agent's path     
                              conflicting_constraint1 = copy(constraints[constraint1[3]])
                              conflicting_constraint1.append(constraint1)
                              # conflicting_constraint1 = updated constraints for conflicting agent 1

                              # node 1 is conflicting agent 1's new path (with the new constraint applied), its new constraints, conflicting agent 1's old  path, its old constraints
                              node1.append(aStar(startPoints[constraint1[3]], endPoints[constraint1[3]], conflicting_constraint1))
                              node1.append(conflicting_constraint1)
                              node2.append(aStar(startPoints[constraint1[3]], endPoints[constraint1[3]], constraints[constraint1[3]]))
                              node2.append(constraints[constraint1[3]])

                         if index == constraint1[4]:
                              # node2 applies conflict1 to the second conflicting agent's path
                              conflicting_constraint2 = copy(constraints[constraint1[4]])
                              conflicting_constraint2.append(constraint1)
                              # conflicting_constraint2 = updated constraints for conflicting agent 2


                              node1.append(aStar(startPoints[constraint1[4]], endPoints[constraint1[4]], constraints[constraint1[4]]))
                              node1.append(constraints[constraint1[4]])
                              node2.append(aStar(startPoints[constraint1[4]], endPoints[constraint1[4]], conflicting_constraint2))
                              node2.append(conflicting_constraint2)
          
             else: 
                    if constraint1[0] == 'edge':
                         node1 = []
                         node2 = []
                         for index, path in enumerate(paths):
                              if index != constraint1[4] and index != constraint1[5]:
                                   node1.append(path)
                                   node1.append(constraints[index])
                                   node2.append(path)
                                   node2.append(constraints[index]) 
                              else:
                                   if index == constraint1[4]:
                         # applies new and old versions of conflicting agent1's paths to nodes 1 and 2    
                                        conflicting_constraint1 = copy(constraints[constraint1[4]])
                                        conflicting_constraint1.append(constraint1)
                                   
                                        node1.append(aStar(startPoints[constraint1[4]], endPoints[constraint1[4]], conflicting_constraint1))
                                        node1.append(conflicting_constraint1)
                                        node2.append(aStar(startPoints[constraint1[4]], endPoints[constraint1[4]], constraints[constraint1[4]]))
                                        node2.append(constraints[constraint1[4]])

                                   if index == constraint1[5]:
                          # applies new and old versions of conflicting agent2's paths to nodes 1 and 2   
                                        conflicting_constraint2 = copy(constraints[constraint1[5]])
                                        conflicting_constraint2.append(constraint1)

                                        node1.append(aStar(startPoints[constraint1[5]], endPoints[constraint1[5]], constraints[constraint1[5]]))
                                        node1.append(constraints[constraint1[5]])
                                        node2.append(aStar(startPoints[constraint1[5]], endPoints[constraint1[5]], conflicting_constraint2))
                                        node2.append(conflicting_constraint2)

          #    print("node1:" , node1)
          #    print("node2:" , node2)

             return node1, node2, None
             


     queue.put(parentNode)
     while queue.empty() == False:
          currNode = queue.get()
          node1, node2, other = gen2Nodes(currNode)
               
          if node1 is None and node2 is None:
               print("Done!")
               return other
          
          if None not in node1:
               queue.put(node1)

          if None not in node2:
               queue.put(node2)


#---------------------------------------------------------------------------------------
# single agent path finding
def aStar(start, end, constraints):
    closedList = [(start)]
    openList = []
    children = []
    cameFrom = {}
    gCosts = {}
    gCosts[start] = 0
    currentNode = start
    while currentNode != end:
        # print("loop entered")
        fCostsOpenList = []
        children = get_adjacent_nodes(currentNode)
        children = [child for child in children if child not in closedList]

        
        for constraint in constraints:
              if constraint[0] == 'vertex': 
                node = constraint[1]
                time = constraint[2]
                currTime = gCosts[currentNode]
                if (time - 1) == currTime:
                        children = [child for child in children if child != node] 

               #  if constraint[1] == end and (time - 1) < currTime:
               #     children = [child for child in children if child != node]

              else: 
                if constraint[0] == 'edge':

                    if currentNode == constraint[1]:
                        node2 = constraint[2]
                        time = constraint[3]
                        currTime = gCosts[currentNode]
                        if (time + 0.5) == (currTime + 1):
                            children = [child for child in children if child != node2]
                    else:
                         if currentNode == constraint[2]:
                            node1 = constraint[1]
                            time = constraint[3]
                            currTime = gCosts[currentNode]
                            if (time + 0.5) == (currTime + 1):
                                children = [child for child in children if child != node1]                          

     #    print(children)
        for child in children:
              gCosts[child] = gCosts[currentNode] + 1
              openList.append((child, fCost(gCosts[child], child, end)))
              cameFrom[child] = currentNode
        
        # finds child node with lowest fCost
        for node in openList:
                fCostsOpenList.append(node[1])

        if not fCostsOpenList:
             print("A* failed!")
             return None
        
        # print("open list: ", openList)
        minIndex = fCostsOpenList.index(min(fCostsOpenList)) 
        for node in openList:
              if node[1] == fCostsOpenList[minIndex]:
                    openList.remove(node)
                    # this now appends the child node with lowest fCost to closed List
                    closedList.append(node[0])
                    currentNode = node[0]
                    # print(node[1])
                    # print(currentNode)
                    break

    # print("closed", closedList)
    # print("open", openList)

    return getPath(cameFrom, start, end)

# create path from closedList
def getPath(cameFrom, start, end):
      path = []
      path.append(end)
      currentNode = end
      while currentNode != start:
        currentNode = cameFrom[currentNode]
        path.insert(0, currentNode)

     #  print("path", path)
      return path 
#---------------------------------------------------------------------------
# new multi agent version
def findFirstConflict(paths):
     first_vertex_conflict = findVertexConflicts(paths)
     first_edge_conflict = findEdgeConflicts(paths)
     first_conflict = None
    
    #  print(first_vertex_conflict)
     if first_vertex_conflict == None:
          first_conflict = first_edge_conflict
     elif first_edge_conflict == None:
          first_conflict = first_vertex_conflict
     elif first_vertex_conflict[2] < first_edge_conflict[3]:
          first_conflict = first_vertex_conflict
     else:
          if first_edge_conflict[3] < first_vertex_conflict[2]:
               first_conflict = first_edge_conflict

     print("first conflict:", first_conflict)
     return(first_conflict)

# -----------------------------------------------------------------
# updated findVertexConflicts for any given number of paths
def findVertexConflicts(paths): 
     #    print(paths) 
     #     for some reason, a node WITH None is being added to the queue... causing the Nonetype error
        longestPath = paths[0]
        for i in range(len(paths)):
             if len(paths[i]) > len(longestPath):
                  longestPath = paths[i]

        for path in paths:
             if path != longestPath:
                  x = len(longestPath) - len(path)
                  
                  for _ in range(x):
                       path.append(path[len(path)-1])
             
     #    print("paths:", paths)

        for index, nodes in enumerate(zip(*paths)):
            seen_nodes = {}
            for path_index, node in enumerate(nodes):
               #  print(node)
                if node in seen_nodes:
                    # print("Conflict:", node, "Index:", index, "Involved paths:", seen_nodes[node], "and", path_index)
                    return ['vertex', node, index, seen_nodes[node], path_index]
                seen_nodes[node] = path_index
# -------------------------------------------------------------------
# update version for multiple agents
def findEdgeConflicts(paths):
          longestPath = paths[0]
          for i in range(len(paths)):
               if len(paths[i]) > len(longestPath):
                    longestPath = paths[i]

          for path in paths:
               if path != longestPath:
                    x = len(longestPath) - len(path)
                    
                    for _ in range(x):
                         path.append(path[len(path)-1])
               
        #   print("paths:", paths)

          for index1, path1 in enumerate(paths):
                for index2, path2 in enumerate(paths):
                    if index2 <= index1:
                         continue
                    for t in range(len(path1)- 1):

                         if path1[t] == path2[t+1] and path1[t+1] == path2[t]:

                              # print(['edge', path1[t], path2[t], (t + 0.5), index1, index2])
                              return(['edge', path1[t], path2[t], (t + 0.5), index1, index2])
                         
# --------------------------------------------------------------------------------------------------------
                              
def findAllConflicts(paths):
        all_conflicts = []
        longestPath = paths[0]
        for i in range(len(paths)):
             if len(paths[i]) > len(longestPath):
                  longestPath = paths[i]

        for path in paths:
             if path != longestPath:
                  x = len(longestPath) - len(path)
                  
                  for _ in range(x):
                       path.append(path[len(path)-1])
             
        # print("paths:", paths)

        for index, nodes in enumerate(zip(*paths)):
            seen_nodes = {}
            for path_index, node in enumerate(nodes):
                if node in seen_nodes:
                    # print("Conflict:", node, "Index:", index, "Involved paths:", seen_nodes[node], "and", path_index)
                    all_conflicts.append(['vertex', node, index, seen_nodes[node], path_index])
                seen_nodes[node] = path_index

        for index1, path1 in enumerate(paths):
                for index2, path2 in enumerate(paths):
                    if index2 <= index1:
                         continue
                    for t in range(len(path1)- 1):

                         if path1[t] == path2[t+1] and path1[t+1] == path2[t]:
                              all_conflicts.append(['edge', path1[t], path2[t], (t + 0.5), index1, index2])

        # print("All Conflicts Found:", all_conflicts)   
        return paths, all_conflicts  

def chooseConflict(paths, heuristic):
    return heuristic(paths)

def mostCrowded(paths):
     lengthened_paths, all_conflicts = findAllConflicts(paths)
     best_constraint = None
     max_value = -1

     for conflict in all_conflicts:
        value = 0
        problem_nodes = get_adjacent_nodes(conflict[1])
        for path in lengthened_paths:
            if conflict[0] == 'vertex':  
               if path[conflict[2]] in problem_nodes:
                    value = value + 1
            if conflict[0] == 'edge':
                 if path[int(conflict[3]- 0.5)] in problem_nodes:
                      value = value + 1
     
        if value > max_value:
            max_value = value
            best_constraint = conflict

     print("best constraint:", best_constraint)
     return best_constraint
    
def randomConflict(paths):
     lengthened_paths, all_conflicts = findAllConflicts(paths)
     if all_conflicts == []:
          return None
     else:
        best_constraint = random.choice(all_conflicts)
        print("best constraint:", best_constraint)
        return best_constraint

def mostConflictingAgent(paths):
     lengthened_paths, all_conflicts = findAllConflicts(paths)
     max_value = 0
     worst_agent_conflicts = []
     best_constraint = None

     for path in lengthened_paths:
        value = 0
        agents_conflicts = []
        for conflict in all_conflicts:
            if conflict[0] == 'vertex':
               if conflict[3] or conflict[4] == lengthened_paths.index(path):
                    value = value + 1
                    agents_conflicts.append(conflict)
            if conflict[0] == 'edge':
                 if conflict[4] or conflict[5] == lengthened_paths.index(path):
                      value = value + 1
                      agents_conflicts.append(conflict)
                 
        if value > max_value:
            max_value = value
            worst_agent_conflicts = agents_conflicts
            best_constraint = worst_agent_conflicts[0]

     print("best constraint:", best_constraint)
     return best_constraint
               
# -----------------------------------------------------------------------------------------------------------------
# code for running experiences with randomly generated instances
findFirst_durations_per_agent = {} 
mostCrowded_durations_per_agent = {}
randomConflict_durations_per_agent = {}
mostConflictingAgent_per_agent = {}


def solveProblems(number_exeriments, agents):
     durations1 = []
     durations2 = []
     durations3 = []
     durations4 = []


     for _ in range(number_exeriments):
          starts, ends = genInstances(agents)

          print("starts: ", starts)
          print("ends: ", ends)

     # findFirstConflict
          start_time = time.time()
          CBS(starts, ends, findFirstConflict)
          end_time = time.time()
          durations1.append((end_time - start_time))
          # average_first = np.mean(durations1)
          # std_first = np.std(durations1)

          if _ == number_exeriments - 1:
               findFirst_durations_per_agent[agents] = durations1

     # print(findFirst_durations_per_agent)
     
     # mostCrowded
          start_time = time.time()
          CBS(starts, ends, mostCrowded)
          end_time = time.time()
          durations2.append((end_time-start_time))
          # average_crowded = np.mean(durations2)
          # std_crowded = np.std(durations2)

          if _ == number_exeriments - 1:
               mostCrowded_durations_per_agent[agents] = durations2

     # randomConflict
          start_time = time.time()
          CBS(starts, ends, randomConflict)
          end_time = time.time()
          durations3.append((end_time-start_time))
          # average_random = np.mean(durations3)
          # std_rand = np.std(durations3)      

          if _ == number_exeriments - 1:
               randomConflict_durations_per_agent[agents] = durations3 

     # mostConflictingAgent
          start_time = time.time()
          CBS(starts, ends, mostConflictingAgent)
          end_time = time.time()
          durations4.append((end_time-start_time))
          # # average_conflicting = np.mean(durations4)
          # # std_most_conflicting = np.std(durations4)

          if _ == number_exeriments - 1:
               mostConflictingAgent_per_agent[agents] = durations4

     # print("mean findFirst: ", average_first)
     # print("mean mostCrowded: ", average_crowded)
     # print("mean random: ", average_random)
     # print("mean mostConflictingAgent: ", average_conflicting)

     # print("standard deviation findFirst: ", std_first)
     # print("standard deviation mostCrowded ", std_crowded)
     # print("standard deviation random: ", std_rand)
     # print("standard deviation mostConflictingAgent: ", std_most_conflicting)

     print("first", findFirst_durations_per_agent)
     print("crowd", mostCrowded_durations_per_agent)
     print("random", randomConflict_durations_per_agent)
     print('most conflicting', mostCrowded_durations_per_agent)

def scalabilityGraph():

     data = {
          'findFirstConflict': findFirst_durations_per_agent,
          'mostCrowded': mostCrowded_durations_per_agent, 
          'randomConflict': randomConflict_durations_per_agent, 
          'mostConflictingAgent': mostConflictingAgent_per_agent
     }

     plt.figure(figsize=(10, 6))

     for heuristic, durations in data.items():
          num_agents = sorted(durations.keys())
          average_durations = [np.mean(durations[agents]) for agents in num_agents]
 

     # Plotting
          plt.plot(num_agents, average_durations, label=heuristic)

     plt.title("Average Duration vs Number of Agents (Different Heuristics)")
     plt.xlabel("Number of Agents")
     plt.ylabel("Average Duration (s)")
     plt.legend()

     plt.show()
     
# ------------------------------------------------------------------------------------------------------------------
# tests:

# graph tests:
solveProblems(20, 2)
solveProblems(20, 3)
solveProblems(20, 4)
solveProblems(20, 5)
# solveProblems(20, 6)
# solveProblems(20, 7)
# solveProblems(20, 8)
# solveProblems(20, 9)
# solveProblems(20, 10)
# solveProblems(20, 11)
# solveProblems(20, 12)
# solveProblems(10, 13)
# solveProblems(10, 14)
# solveProblems(10, 15)
scalabilityGraph()

# solveProblems(3, 2)
# solveProblems(3, 3)
# solveProblems(3, 4)
# scalabilityGraph()
# solveProblems(3, 10)
# solveProblems(3, 11)
# solveProblems(3, 12)
# solveProblems(3, 13)
# solveProblems(3, 14)
# solveProblems(3, 15)

