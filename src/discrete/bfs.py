import numpy as np
from shapely.geometry import LineString, Point, Polygon

class Node: 
    '''Class representing a point in the BFS grid

    Args: 
        x (int): x coordinate of grid point 
        y (int): y coordinate of grid point
        parent (list[int, int]): [x,y] point of parent node
    Returns: 
        Node: An instance of the Node class 
    '''
    def __init__(self, x, y, parent):
        self.x = x
        self.y = y
        self.parent = parent # coordinates of parent node or -1 if start, -2 if goal

def in_window(x, y, window_width, window_height):
    '''Checks if a point, (x,y), is in the defined window

    Args: 
        x (int): x coordinate of point
        y (int): y coordinate of point
        window_width (int): window's size in the x direction
        window_height (int): window's size in the y direction
    Returns: 
        bool: bool variable showing if the point (x,y) is in the window
    '''
    if x < 0 or y < 0 or x >= window_width or y >= window_height:
        return False
    return True

def get_neighbours(node, parents): 
    '''Get neighbouring Nodes for a N,E,S,W grid

    Args: 
        node (Node): input Node 
        parents (numpy array): ndarray showing parents of points in bfs grid
    '''
    positions = [[-1,0], [1, 0], [0, -1], [0, 1]] # N E S W grid 
    
    x = node.x
    y = node.y 
    
    neighbours = []

    for pos in positions: 
        # Check in bounds 
        if not in_window(x+pos[0], y+pos[1], parents.shape[0], parents.shape[1]):
            continue

        if parents[x+pos[0], y+pos[1]] != None: 
            continue

        node = Node(x+pos[0], y+pos[1], [x,y])
        neighbours.append(node)

    return neighbours

def mark_points_in_Circle(grid, cx, cy, r):
    '''Marks points contained within a Circle on a bfs grid as True

    Args: 
        grid (numpy array): input grid to be marked 
        cx (int): x coordinate of the circle's centre
        cy (int): y coordinate of the circle's centre
        r (int): circle radius
    Returns: 
        grid (numpy array): updated grid 
    '''
    
    for i in range(cx - r, cx + r + 1):
        for j in range(cy - r, cy + r + 1):

            # Check in bounds 
            if not in_window(i, j, grid.shape[0], grid.shape[1]):
                continue

            if (((i-cx)**2 + (j-cy)**2) <= r**2):
                grid[i,j] = True

    return grid

def mark_points_in_Rectangle(grid, rx1, ry1, rx2, ry2):
    '''Marks points contained within a Rectangle on a bfs grid as True

    Args: 
        grid (numpy array): input grid to be marked 
        rx1 (int): x coordinate of first point defining a rectangle
        ry1 (int): y coordinate of first point defining a rectangle
        rx2 (int): x coordinate of second point defining a rectangle
        ry2 (int): y coordinate of second point defining a rectangle
    Returns: 
        grid (numpy array): updated grid 
    '''
    
    for i in range(rx1, rx2 + 1): 
        for j in range(ry1, ry2 + 1):

            # Check in bounds 
            if not in_window(i, j, grid.shape[0], grid.shape[1]):
                continue
            
            grid[i,j] = True
    
    return grid

def get_path_from_goal(gx, gy, parent_grid): 
    '''Gets bfs path from goal point to the start point 

    Args: 
        gx (int): x coordiante of goal point 
        gy (int): y coordiante of goal point 
        parent_grid (numpy array): grid showing parents of each point
    Returns: 
        path (list[list]): List of coordinates along path from goal to start
    '''
    path = []
    parent = parent_grid[gx, gy]

    path.append([gx,gy])

    while True:
        if parent == [-1]:
            break 
        path.append(parent)

        parent = parent_grid[parent[0], parent[1]]

    return path

    # Make format match rrt 

    # Use list of dictionaries 
    ret_path = []

    for pt in path: 
        parent = parent_grid[pt[0], pt[1]]

        if parent == [-1]: 
            index = -1
        else: 
            index = path.index(parent)

        point = {'x': pt[0], 'y': pt[1], 'parentIndex': index}
        ret_path.append(point)


    return ret_path

class BFS:
    '''Class representing a BFS path-finding algorithm 
    
    Args: 
        payload: input payload 
    Returns: 
        bfs (BFS): An instance of the BFS Class

    '''
    def __init__(self, payload):
        width =  payload['width']
        height = payload['height']
        step_size = payload['step_size']
        grid_width = int(width/step_size)
        grid_height = int(height/step_size)

        start = np.array(payload['start'], dtype=int)
        goal = np.array(payload['goal'], dtype=int)
        goal_radius =  payload['goalRadius']
        
        obstacles = payload['obstacles']

        # Define Grid as np arrays 
        visited_grid = np.zeros((grid_width, grid_height), dtype=bool)
        blocked_grid = np.zeros((grid_width, grid_height), dtype=bool)
        goal_grid = np.zeros((grid_width, grid_height), dtype=bool)
        parent_grid = np.empty((grid_width, grid_height), dtype=object)


        # Mark start position
        start = [int(start[0]/step_size), int(start[1]/step_size)]
        parent_grid[start[0], start[1]] = [-1]

        # Mark goal points 
        goal_grid = mark_points_in_Circle(  goal_grid, 
                                            int(goal[0]/step_size),    
                                            int(goal[1]/step_size), 
                                            int(goal_radius/step_size))
        
        # Mark blocked points 
        for obstacle in obstacles: 
            if obstacle['shape'] == "rectangle":
                obs = np.array(obstacle['definition']) / step_size
                blocked_grid = mark_points_in_Rectangle(blocked_grid,
                                                        int(obs[0]), 
                                                        int(obs[1]),
                                                        int(obs[2]),
                                                        int(obs[3]))

            if obstacle['shape'] == "circle":
                cx,cy = obstacle['definition'][0] / step_size ,obstacle['definition'][1] / step_size
                r = np.ceil((obstacle['definition'][2]*2) / step_size)
                blocked_grid = mark_points_in_Circle(blocked_grid, int(cx), int(cy), int(r))
        

        start_node = Node(int(start[0]), int(start[1]), -1)

        nodes = []
        nodes.append(start_node)
        visited_order = []

        self.blocked_grid = blocked_grid
        self.goal_grid = goal_grid

        path_found = False
        
        while len(nodes) != 0 and not path_found: 

            node = nodes.pop(0)

            # If visited, continue
            if visited_grid[node.x, node.y] == True: continue

            # Get neighbours    
            neighbours = get_neighbours(node, parent_grid)

            # Mark visited and add to visited list 
            visited_grid[node.x, node.y] = True
            visited_order.append([node.x, node.y])

            # Check if goal 
            if goal_grid[node.x, node.y]:
                path = get_path_from_goal(node.x, node.y, parent_grid)
                self.path = path
                path_found = True

            # For each neighbour 
            for neighbour in neighbours:
                
                nx = neighbour.x
                ny = neighbour.y

                # Check if visited or blocked
                if blocked_grid[nx, ny] or visited_grid[nx, ny]: 
                    continue

                # Mark parent 
                parent_grid[nx, ny] = neighbour.parent
                    
                # Add to queue 
                nodes.append(neighbour)

        # Put data into desired format for the front end 
        points_list = []

        for pt in visited_order: 
            parent = parent_grid[pt[0], pt[1]]

            if parent == [-1]: 
                index = -1
            else: 
                index = visited_order.index(parent)

            point = {'x': pt[0]*step_size, 'y': pt[1]*step_size, 'parentIndex': index}
            points_list.append(point)

        self.points_list = points_list

