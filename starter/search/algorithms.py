import heapq
import math

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost

def dijkstras(start_state, goal_state, gridded_map):
    """
    Implements Dijkstra's algorithm to find the lowest-cost path from start_state to goal_state
    on the given gridded_map. The function returns a tuple containing the path found (as a list of states),
    the cost of the path, and the number of nodes expanded during the search.
    """
    # Initialize the OPEN list as a priority queue
    open_heap = []
    

    #Hash for best g values seen so far
    best_g = {}

    #Set up the start state
    start_state.set_g(0.0)
    start_state.set_cost(0.0)
    start_state.set_parent(None)

    #Push it into OPEN 
    heapq.heappush(open_heap, start_state)

    best_g[start_state.state_hash()] = 0.0
    expanded_nodes = 0  

    #main loop 
    while open_heap:
        #Pop the best node i.e the node with the lowest cost
        current_state = heapq.heappop(open_heap)
        
        #Check if we have already found a better path to this statePre
        ch = current_state.state_hash()
        if current_state.get_g() > best_g.get(ch, float('inf')):
            continue

        #Check if we reached the goal
        if current_state == goal_state:
            #Reconstruct the path
            path = []
            total_cost = current_state.get_g()

            while current_state is not None:
                path.append(current_state)
                current_state = current_state.get_parent()
            path.reverse()
            return path, total_cost, expanded_nodes
        
        #Expand by generating successors
        expanded_nodes += 1

        for child in gridded_map.successors(current_state):
            #Read the g value of the child
            new_g = child.get_g()

            #If this is a better path then we should update it
            hash_value = child.state_hash()
            if new_g < best_g.get(hash_value, float('inf')):
                #Update best g value
                best_g[hash_value] = new_g

                #Set the parent of the child to the current state you are expanding
                #this is because we found a better path to the child so we are making
                #a way to construct the path later
                child.set_parent(current_state)
                
                #Update the g of the child because we found a better path to it
                child.set_g(new_g)

                #Update the cost of the child
                child.set_cost(new_g)  

                #Push the child into OPEN so that we can expand it later
                heapq.heappush(open_heap, child)

                
    
    #If we get here, no path was found
    return None, -1, expanded_nodes

def a_star(start_state, goal_state, gridded_map, w=1.0):
    """
    Implements the A* algorithm to find the lowest-cost path from start_state to goal_state
    on the given gridded_map. The function returns a tuple containing the path found (as a list of states),
    the cost of the path, and the number of nodes expanded during the search.
    """
     # Initialize the OPEN list as a priority queue
    open_heap = []
    

    #Hash for best g values seen so far
    best_g = {}


    #Set up the start state
    start_state.set_g(0.0)
    start_state.set_cost(heuristic(start_state, goal_state))
    start_state.set_parent(None)

    #Push it into OPEN 
    heapq.heappush(open_heap, start_state)

    best_g[start_state.state_hash()] = 0.0
    expanded_nodes = 0  

    #main loop 
    while open_heap:
        #Pop the best node i.e the node with the lowest cost
        current_state = heapq.heappop(open_heap)
        
        #Check if we have already found a better path to this statePre
        ch = current_state.state_hash()
        if current_state.get_g() > best_g.get(ch, float('inf')):
            continue

        #Check if we reached the goal
        if current_state == goal_state:
            #Reconstruct the path
            path = []
            #cost = distance from start to goal
            total_cost = current_state.get_g() 

            while current_state is not None:
                path.append(current_state)
                current_state = current_state.get_parent()
            path.reverse()
            return path, total_cost, expanded_nodes
        
        #Expand by generating successors
        expanded_nodes += 1

        for child in gridded_map.successors(current_state):
            #Read the g value of the child
            new_g = child.get_g()

            #If this is a better path then we should update it
            hash_value = child.state_hash()
            if new_g < best_g.get(hash_value, float('inf')):
                #Update best g value
                best_g[hash_value] = new_g

                #Set the parent of the child to the current state you are expanding
                #this is because we found a better path to the child so we are making
                #a way to construct the path later
                child.set_parent(current_state)
                
                #Update the cost of the child because we found a better path to it
                h = heuristic(child, goal_state)
                child.set_cost(new_g + h*w)
                

                #Push the child into OPEN so that we can expand it later
                heapq.heappush(open_heap, child)

                
    
    #If we get here, no path was found
    return None, -1, expanded_nodes

def heuristic(state, goal_state):
    """
    Computes the Euclidean distance between the given state and the goal_state.
    This function is used as the heuristic for the A* algorithm.
    """
    #Based on assignment description, dx = |x - x_goal| and dy = |y - y_goal|
    #then h = 1.5 * min(dx,dy) + |dx - dy|
    dx = abs(state.get_x() - goal_state.get_x())    
    dy = abs(state.get_y() - goal_state.get_y())
    h = 1.5 * min(dx, dy) + abs(dx - dy)
    return h
