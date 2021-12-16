"""
This class implements a KDTreeNode data-structure which is the data-structure of 
nodes in the KDTree class.

Helper functions in this class can return the distance to another node, distance to 
the splitting hyperplane, comparisons (and operators) between nodes. 

"""

import numpy as np


class KDTreeNode:
    '''
    Args: 
        point (np.ndarray): the position of the node (x,y)
        depth (integer): the depth of the node in the tree

    Return:
        KDTreeNode: An instance of the KDTreeNode class

    '''
    def __init__(self, point=np.array((0, 0)), depth=0):
        self.p = np.array(point)
        self.depth = depth

        # The dimension of the splitting plane (i.e. 0 -> the x dimension, 1-> y )
        self.axis = depth % len(point)

        # The children nodes
        self.left = None
        self.right = None

        # I will define the parent as the closest neighbor to this node
        # at the time this node was added to the tree
        self.parent = None
        # The index in the kdtree array representation. Only the root node should have a value of -1 as it has no parents
        self.index = 0 
        self.isOnGoalPath = False

    def set_depth(self, depth):
        '''Sets the depth of the node in the tree to the desired depth 

        Args: 
            self (KDTree): a KDTree object
            depth (integer): desired tree depth
            
        Return:
            None

        '''
        self.depth = depth
        self.axis = depth % len(self.p)

    def distance_to_node(self, other):
        '''Returns the distance between the current node and the input node 

        Args: 
            self (KDTreeNode): a KDTreeNode object
            other (KDTreeNode): the KDTreeNode object we want to find the distance to
            
        Return:
            float: the euclidean distance between the two nodes 

        '''

        return np.linalg.norm(self.p - other.p)
    
    def distance_to_point(self, other):
        '''Returns the distance between the current node and the input node 

        Args: 
            self (KDTreeNode): a KDTreeNode object
            other (np.ndarray): the point we want to find the distance to
            
        Return:
            float: the euclidean distance between the node and the point 

        '''
        
        return np.linalg.norm(self.p - other)

    def distance_to_splitting_plane(self, other):
        '''Returns the distance between the current node's splitting axis 
        and the input points value at that axis, i.e., if the splitting plane is 
        the x axis, compare the node's x value to the point's x value

        Args: 
            self (KDTreeNode): a KDTreeNode object
            other (np.ndarray): the point we want to find the distance to
            
        Return:
            float: the euclidean distance between the point and the splitting axis 

        '''

        axis = self.axis
        return abs(self.p[axis] - other[axis])

    def compare_to_point(self, point):
        ''' Computes the difference between the point's value on the splitting axis 
        and the current nodes value on that splitting axis. 

        Args: 
            self (KDTreeNode): a KDTreeNode object
            other (np.ndarray): the point we want to compare to
            
        Return:
            float: difference in values between self and other on a particular axis

        '''

        axis = self.axis
        return self.p[axis] - point[axis]

    def node_to_dict(self):
        ''' Returns a dictionary representation of a KDTree Node object 

        Args: 
            self (KDTreeNode): a KDTreeNode object
            
        Return:
            Dictionary: returns the x position, y position and parent of the KDTreeNode
            in a dictionary

        '''

        parentIndex = -1
        if self.parent is not None:
            parentIndex = self.parent.index
        return {
            "x":self.p[0],
            "y":self.p[1],
            "parentIndex": parentIndex}

    def __lt__(self, other):
        ''' Operator overload for the < (less than). Compares the current node and 
        another node's values at the dimension of the other node (i.e. if the 
        other node is at dimension 0, compare the x values) 

        Args: 
            self (KDTreeNode): a KDTreeNode object
            other (KDTree Node): a KDTreeNode object that we want to compare to
            
        Return:
            Boolean: True if self is less than or equal to other, False otherwise

        '''

        axis = other.axis
        p = self.p
        return p[axis] <= other.p[axis]

    def __gt__(self, other):
        ''' Operator overload for the > (greater than). Compares the current node and 
        another node's values at the dimension of the other node (i.e. if the 
        other node is at dimension 0, compare the x values) 

        Args: 
            self (KDTreeNode): a KDTreeNode object
            other (KDTree Node): a KDTreeNode object that we want to compare to
            
        Return:
            Boolean: True if self is greater than other, False otherwise

        '''

        axis = self.axis
        p = self.p
        return p[axis] > other.p[axis]
    
    def __eq__(self, other):
        ''' Operator overload for the = (equal to). Compares the current node and 
        another node's values at the dimension of the other node (i.e. if the 
        other node is at dimension 0, compare the x values) 

        Args: 
            self (KDTreeNode): a KDTreeNode object
            other (KDTree Node): a KDTreeNode object that we want to compare to
            
        Return:
            Boolean: True if self is equal to other, False otherwise

        '''

        return self.p == other.p

    def __repr__(self):
        ''' A method to represent the KDTreeNode object as a string when print() is 
        called on the object

        Args: 
            self (KDTreeNode): a KDTreeNode object

        Return:
            String: A string representation of the KDTreeNode

        '''
        
        return "Point: {}, Depth: {}, Splitting Axis: {}".format(self.p, self.depth, self.axis)

    def set_goal_path(self):
        ''' Computes the path from self to every parent until root. This is called on 
        the goal node to return the path to the goal. It sets every node on the path 
        as being a part of the Goal Path. 

        Args: 
            self (KDTreeNode): a KDTreeNode object

        Return:
            None

        '''

        cur_node = self
        while cur_node is not None:
            cur_node.isOnGoalPath = True
            cur_node = cur_node.parent

if __name__ == '__main__':
    a = KDTreeNode((-1,-1),1)
    b = KDTreeNode((2,2),1)
    
    print(b.distance_to_splitting_plane((10,11)))