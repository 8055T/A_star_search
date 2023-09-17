# A_star_search
Modifying BFS code into A* search using heuristic (h2) Manhattan distance
import numpy as np
#################### BFS ##############################
####################    Node definitions   ###########################
class Node:
    def __init__(self, state, action, parent, cost):
        self.s = state
        self.a = action
        self.p = parent
        self.c = cost
        self.expand = 0
       
    def printstate(self):
        print(self.s)
    
    def printaction(self):
        print(self.a) 

################## Action functions ##################################
def swap(array,p1,p2,p3,p4):
    temp=array[p1,p2]
    array[p1,p2]=array[p3,p4]
    array[p3,p4]=temp
    return array

def goup(value):
    value = np.array(value)
    a=(np.where(value==0))
    if a[0][0]-1<0:
        return value
    else:
        ans=swap(value,a[0][0],a[1][0],a[0][0]-1,a[1][0])
        return ans

def godown(value):
    value = np.array(value)
    a=(np.where(value==0))
    if a[0][0]+1==value.shape[1]:
        return value
    else:
        ans=swap(value,a[0][0],a[1][0],a[0][0]+1,a[1][0])
        return ans

def goleft(value):
    value = np.array(value)
    a=(np.where(value==0))
    if a[1][0]-1<0:
        return value
    else:
        ans=swap(value,a[0][0],a[1][0],a[0][0],a[1][0]-1)
        return ans

def goright(value):
    value = np.array(value)
    a=(np.where(value==0))
    if a[1][0]+1==value.shape[1]:
        return value
    else:
        ans=swap(value,a[0][0],a[1][0],a[0][0],a[1][0]+1)
        return ans


##########################   Main   #######################################
maxdepth = 9999
#start = np.array([[1,2,3],[4,5,6],[0,7,8]])  # change your starting here
start = np.array([[4,1,0],[7,2,3],[5,8,6]])   # change your starting here
goal = np.array([[1,2,3],[4,5,6],[7,8,0]])

root = Node(start,0,0,0)
nodelist = [root]
costlist = np.array([0])
nodecount = 1

found = None
while found==None:
    # Search for a node to expand
    breadth = np.argmin(costlist)
    costlist[breadth] = maxdepth        # Eliminate found node from the list
    parent = nodelist[breadth]
        
    # Expand
    parent.expand = 1   # Mark expanded
    depth = parent.c + 1
    up = Node(goup(parent.s), 'up', parent, depth)
    down = Node(godown(parent.s), 'down', parent, depth)
    left = Node(goleft(parent.s), 'left', parent, depth)
    right = Node(goright(parent.s), 'right', parent, depth)
    nodelist.extend([up,down,left,right])
    costlist = np.append(costlist,[depth,depth,depth,depth])
    
    # Check if a solution is found
    if sum(sum(up.s != goal)) == 0:
        found = up
    if sum(sum(down.s != goal)) == 0:
        found = down
    if sum(sum(left.s != goal)) == 0:
        found = left
    if sum(sum(right.s != goal)) == 0:
        found = right
        
    nodecount = nodecount + 4

# Print solution        
print('Solution found in ' + str(found.c) + ' moves')
print('Generated ' + str(nodecount) + ' nodes')
solution = []
while found.c > 0 :
    solution.append(found)
    found = found.p

print(start)
for i in range(len(solution)-1,-1,-1):
    solution[i].printaction()
    solution[i].printstate()

#I have to modify above BFS search code into A* search using heuristic (h2) Manhattan distance. Help me seniors to solve this problem please!    
# Modify the code for A* search with Manhattan distance heuristic (h2)
from queue import PriorityQueue

def manhattan_distance(state, goal):
    # Calculate the Manhattan distance between two states
    distance = 0
    for i in range(state.shape[0]):
        for j in range(state.shape[1]):
            if state[i, j] != 0:
                goal_position = np.where(goal == state[i, j])
                distance += abs(i - goal_position[0]) + abs(j - goal_position[1])
    return distance

def astar_search(start, goal):
    maxdepth = 9999
    root = Node(start, 0, 0, 0)
    nodelist = PriorityQueue()
    nodelist.put((0, root))  # Priority queue sorted by f(n) = g(n) + h(n), where g(n) is depth, h(n) is Manhattan distance
    found = None
    nodecount = 1

    while not nodelist.empty():
        _, current_node = nodelist.get()
        if np.array_equal(current_node.s, goal):
            found = current_node
            break

        if current_node.c < maxdepth:
            depth = current_node.c + 1
            actions = ['up', 'down', 'left', 'right']
            for action in actions:
                new_state = None
                if action == 'up':
                    new_state = goup(current_node.s)
                elif action == 'down':
                    new_state = godown(current_node.s)
                elif action == 'left':
                    new_state = goleft(current_node.s)
                elif action == 'right':
                    new_state = goright(current_node.s)

                if new_state is not None:
                    new_cost = depth
                    priority = new_cost + manhattan_distance(new_state, goal)
                    new_node = Node(new_state, action, current_node, depth)
                    nodelist.put((priority, new_node))
                    nodecount += 1

    return found, nodecount

# Testing A* search
start = np.array([[4, 1, 0], [7, 2, 3], [5, 8, 6]])
goal = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 0]])

found, nodecount = astar_search(start, goal)

# Print solution
if found is not None:
    print('A* Search:')
    print('Solution found in ' + str(found.c) + ' moves')
    print('Generated ' + str(nodecount) + ' nodes')
    solution = []
    while found.c > 0:
        solution.append(found)
        found = found.p

        print(start)
    for i in range(len(solution) - 1, -1, -1):
        solution[i].printaction()
        solution[i].printstate()
else:
    print("A* Search did not find a solution.")
