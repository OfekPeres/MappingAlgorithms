"""
This class implements the KDTree data-structure which is an efficient organization
of points in a k-dimensional space, where each level of the tree is split into 
a hyperplane that divides the points into two partitions. 

Allows functionality to insert points into the tree, return the points in order, 
and returns the nearest neighbor of a point given another point. 

"""

from .KDTreeNode import KDTreeNode
from typing import List, Text
from pprint import pprint

import matplotlib.pyplot as plt

class KDTree:
    '''
    Args: 
        point (np.ndarray): the starting point for the KD Tree

    Return:
        An instance of the KDTree class

    '''
    def __init__(self, point=(0, 0)):
        self.root = KDTreeNode(point, 0)
        self.TreeList = [self.root.node_to_dict()]

    def Insert(self, point):
        '''Inserts a new KDTreeNode into the KDTree thru an iterative approach
        returns the node that was created

        Args: 
            self (KDTree): a KDTree object
            point (np.ndarray): the (k-dimensional) point to be inserted into the KDTree
        
        Return:
            KDTreeNode: a KDTree node of the new point inserted

        '''
        depth = 0
        current_node = self.root
        new_node = KDTreeNode(point, depth)
        new_node.parent = self.NearestNeighbor(point)
        new_node.index = len(self.TreeList)
        self.TreeList.append(new_node.node_to_dict())
        while True:
            # Case where new node is greater, go to the right
            if current_node < new_node:
                # If a node exists to the right, jump to it
                if current_node.right is not None:
                    current_node = current_node.right
                # If there is no right node, then place your new node here
                else:
                    new_node.set_depth(depth + 1)
                    current_node.right = new_node
                    break
            # Case where new node is smaller, go to the left
            else:
                if current_node.left is not None:
                    current_node = current_node.left
                else:
                    new_node.set_depth(depth + 1)
                    current_node.left = new_node
                    break
            depth = depth + 1
            new_node.set_depth(depth)
        return new_node

    def __GetPointsInOrderTraversal(self, node, output_list: List):
        '''Takes in a list to populate with all of the nodes in the tree in 
        ascending order. This method recursively does in order traversal

        Args: 
            self (KDTree): a KDTree object
            node (KDTreeNode): the start node for the recursive calls
            output_list (List): to write in the points
        
        Return:
            None 

        '''

        # base case
        if node is None:
            return
        # keep going through tree and add in node to the output list
        self.__GetPointsInOrderTraversal(node.left, output_list)
        output_list.append(node)
        self.__GetPointsInOrderTraversal(node.right, output_list)

    def GetAllNodes(self):
        '''Returns a list of all nodes in the tree using an In Order Traversal. Calls
        __GetPointsInOrderTraversal. 

        Args: 
            self (KDTree): a KDTree object
        
        Return:
            List: a list of nodes in the tree

        '''

        nodes = []
        self.__GetPointsInOrderTraversal(self.root, nodes)
        return nodes

    def CloserKDTreeNode(self, target, p1: KDTreeNode, p2: KDTreeNode):
        ''' Takes in target and returns the tree node closer to the point
        
        Args:
            self (KDTree): a KDTree object
            target (np.ndarray): a k-dimensional point
            p1 (KDTreeNode): a tree node
            p2 (KDtreeNode): another tree node

        Returns: 
            KDTreeNode: the tree node (p1 or p2) that is closest to the target
        
        '''
        if p1 is None:
            return p2
        if p2 is None:
            return p1

        d1 = p1.distance_to_point(target)
        d2 = p2.distance_to_point(target)

        if d1 < d2:
            return p1
        else:
            return p2

    def __NearestNeighbor(self, root: KDTreeNode, point):
        ''' Returns the KDTree node that is closest to a given point (private method
        for recursion)
        
        Args:
            self (KDTree): a KDTree object
            root (KDTreeNode): the root of the KDTree
            point (np.ndarray): the point that is being queried

        Returns: 
            KDTreeNode: the tree node in the KDTree that is closest to the target
        
        '''

        if root is None:
            return None

        next_branch = None
        opposite_branch = None

        if root.compare_to_point(point) > 0:
            next_branch = root.left
            opposite_branch = root.right
        else:
            next_branch = root.right
            opposite_branch = root.left

        # Figure out what node is closest to our search point.
        # Compares the best result from the new branch to the current node we
        # are at
        best: KDTreeNode = self.CloserKDTreeNode(
            point, self.__NearestNeighbor(next_branch, point), root)
        # If the splitting plane is closer than the current best, search the other
        # branch in case it contains a closer node
        if best.distance_to_point(point) > root.distance_to_splitting_plane(
                point):
            best = self.CloserKDTreeNode(
                point, self.__NearestNeighbor(opposite_branch, point), best)

        return best

    def NearestNeighbor(self, point):
        ''' Returns the KDTree node that is closest to a given point (public method)
        
        Args:
            self (KDTree): a KDTree object
            point (np.ndarray): the point that is being queried

        Returns: 
            KDTreeNode: the tree node in the KDTree that is closest to the target
        
        '''
        return self.__NearestNeighbor(self.root, point)

    def GetTreeAsList(self):
        ''' Returns the KDTree as a list
        
        Args:
            self (KDTree): a KDTree object

        Returns: 
            List: return the points in the tree as a list

        '''
        return self.TreeList


if __name__ == '__main__':
    tree = KDTree((11, 10))
    tree.Insert((4, 7))
    node = tree.Insert((16, 10))
    tree.Insert((7, 13))
    tree.Insert((14, 11))
    tree.Insert((9, 4))
    tree.Insert((15, 3))
    node.set_goal_path()
    outputList = tree.GetAllNodes()

    pivot = [14,9]
    print(tree.NearestNeighbor(pivot))
    pprint(tree.GetTreeAsList())

    
    
