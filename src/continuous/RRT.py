"""
RRT Docs - TODO: Complete me!
"""
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from ..utils.KDTree import KDTree
import matplotlib.pyplot as plt
def Steer(x_random, x_nearest, d_max): 
    """
    ### Moves towards x_random up to max distance d_max
    x_random: randomly positioned point within the "map"
    x_nearest: the current nearest point in the RRT
    d_max: the "step" size from each node to node in RRT
    """
    distance = np.linalg.norm(x_random - x_nearest)
    if distance > d_max:
        direction = (x_random - x_nearest)/distance
        x_new = x_nearest + direction*d_max 
    else: 
        x_new = x_random; 
    return x_new



def CreatePolygon(obstacle): 
    """
    ### Takes in square obs defined as [x_lower, y_lower, x_upper, y_upper] and returns a shapely polygon object
    """
    p1 = (obstacle[0], obstacle[1])
    p2 = (obstacle[0], obstacle[3])
    p3 = (obstacle[2], obstacle[3])
    p4 = (obstacle[2], obstacle[1])
    obs_polygon = Polygon([p1,p2,p3,p4])
    return obs_polygon

def IsCollision(x_new, x_nearest, obstacles, eps): 
    """
    ### Checks if adding the new point would cause a collision with an obstacle
    """
    end = Point(x_new[0], x_new[1])
    start = Point(x_nearest[0], x_nearest[1])
    path = LineString([start, end])
    for obstacle in obstacles:
        if obstacle['shape'] == "rectangle":
            obs_cur = np.array(obstacle['definition']) + np.array([-1,-1,1,1])*eps
            obs_polygon = CreatePolygon(obs_cur)
            # check if path intersects 
            collision = path.intersects(obs_polygon)
            if collision: 
                return True
        elif obstacle['shape'] == "circle":
            cx,cy = obstacle['definition'][0],obstacle['definition'][1]
            r = obstacle['definition'][2]
            p = Point(cx,cy)
            circle = p.buffer(3).boundary
            # Check that the end point is not within the circle
            if end.distance(p) < r:
                return True
            if start.distance(p) < r:
                return True
            # Use Shapely to do circle line segment intersection
            collision = path.intersects(circle)
            if collision:
                return True
    return False

def GenRandomPoint(x_min = 0, x_max = 100, y_min = 0, y_max = 50):
    """
    Takes in the x bounds and then the y bounds for the "map" of the RRT 
    @param x_min: the lower x bound of the map
    @param x_max: the upper x bound of the map
    @param y_min: the lower x bound of the map
    @param y_max: the upper x bound of the map
    @returns a random point within the bounds of the map
    """
    x = np.random.uniform(x_min, x_max)
    y = np.random.uniform(y_min, y_max)
    return [x,y]
class RRT:
    def __init__(self, payload):
        start = np.array(payload['start'])
        goal = np.array(payload['goal'])
        goal_radius =  payload['goalRadius']
        obstacles = payload['obstacles']
        d_max = payload['d_max']
        width =  payload['width']
        height = payload['height']
        kdtree = KDTree(start)
        start = np.array(start) 
        goal = np.array(goal)
        dist_to_goal = np.linalg.norm(start - goal)
        eps = 0.5 # safety margin around obstacle
        # Continue searching for points until arrival at goal area
        while dist_to_goal > goal_radius: 
            # generate a random point 
            x_rand = GenRandomPoint(0, width, 0, height)
            # get nearest point in graph from x_rand
            node_nearest = kdtree.NearestNeighbor(x_rand)
            x_nearest = node_nearest.p
            # generate x_new based on random point using steer
            x_new = Steer(x_rand,x_nearest,d_max)
            # check if path to x_new collides with obstacle
            collision = IsCollision(x_new, x_nearest, obstacles, eps)
            # if no collision add it to tree 
            if not collision: 
                x_last = kdtree.Insert(x_new)
                dist_to_goal = np.linalg.norm(x_new - goal)
        self.RRTTree = kdtree
        self.RRTPts = kdtree.GetTreeAsList()
        self.obstacles = obstacles
        self.Target_Node = x_last
        self.goal = goal
        self.goal_radius = goal_radius
    
    def PlotRRT(self):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        pts = self.RRTPts
        for pt in pts[1:]:
            plt.scatter(pt["x"], pt["y"], c="blue", s=1)
            xParent, yParent = pts[pt["parentIndex"]]["x"],pts[pt["parentIndex"]]["y"]
            # print("({},{})".format(xParent,yParent))
            x = [pt["x"], xParent]
            y = [pt["y"], yParent]
            plt.plot(x,y, c="green")
        goalpt = self.RRTPts[self.Target_Node.index]
        cur_pt = goalpt
        while cur_pt["parentIndex"] != -1:
            xParent, yParent = self.RRTPts[cur_pt["parentIndex"]]["x"],self.RRTPts[cur_pt["parentIndex"]]["y"]
            x = [cur_pt["x"], xParent]
            y = [cur_pt["y"], yParent]
            plt.plot(x,y, c="blue")
            cur_pt = self.RRTPts[cur_pt["parentIndex"]]
            
        for obstacle in self.obstacles: 
            if obstacle['shape'] == 'rectangle':
                obs_cur_poly = CreatePolygon(obstacle['definition'])
                x,y = obs_cur_poly.exterior.xy
                ax.fill(x,y, color = 'red')
            if obstacle['shape'] == 'circle':
                obstacle_circle = plt.Circle(obstacle['definition'][:2], obstacle['definition'][2], color='red')
                ax.add_patch(obstacle_circle)

        
        goal_circle = plt.Circle(self.goal, self.goal_radius, color='blue')
        ax.add_patch(goal_circle)
        
        plt.show()
    def GetRRTPayload(self):
        payload = {}
        payload['goal'] = self.goal
        payload['goalRadius'] = self.goal_radius
        payload['obstacles'] = self.obstacles
        payload['points'] = self.RRTPts
        payload['targetNodeIndex'] = self.Target_Node.index
        return payload


