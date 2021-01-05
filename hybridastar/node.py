# -*- coding: utf-8 -*-

import heapq
import numpy as np
                

class Node:
    
    def __init__(self,cstate,dstate,gcost=0,hcost=0,parent=None,history=None):
        """
        Initialize a node.

        Parameters
        ----------
        cstate : tuple/numpy array of float
            Continous state (x,y,theta).
        dstate : tuple/numpy array of float
            Discrete state (x,y,theta).
        gcost : float, optional
            G-cost of the node. The default is 0.
        hcost : float, optional
            H-cost of the node. The default is 0.
        parent : Node, optional
            Parent node. The default is None.
        history : list, optional
            Path to the parent node. The default is None.

        Returns
        -------
        None.

        """
        self._validate_node(cstate,dstate,gcost,hcost,parent,history)
        
        self.cstate = cstate
        self.dstate = dstate
        self.gcost = gcost
        self.hcost = hcost
        self._parent = parent
        self._history = history
        
    def computeFCost(self):
        return self.gcost + self.hcost
    
    def trace(self):
        path = []
        current = self
        while current!=None and current._history!=None:
            for pose in current._history:
                path.append(pose)
            current = current._parent
        return path
    
    def drawPath(self,ax):
        x,y,_ = self._history.T
        ax.plot(x,y,'k')
        
    def _validate_node(self,cstate,dstate,gcost,hcost,parent,history):
        if not isinstance(cstate,(tuple,list,np.ndarray)):
            raise TypeError("cstate must be a tuple, list or numpy array.")
        if not isinstance(dstate,(tuple,list,np.ndarray)):
            raise TypeError("dstate must be a tuple, list or numpy array.")
        if len(cstate)!=3 or len(dstate)!=3:
            raise ValueError("cstate or dstate must be 3 element tuples, lists or numpy arrays")
        if parent is not None and isinstance(parent,(Node)) is False:
            raise TypeError("parent must be a Node object or set to None.")
        if history is not None:
            if not isinstance(history,(list)):
                raise TypeError("history must be a list or set to None.")
            for pose in history:
                if not isinstance(pose,(tuple,list,np.ndarray)):
                    raise TypeError("Each element in history must be a tuple, list or numpy array.")
                if len(pose)!=3:
                    raise ValueError("Each element in history must have 3 elements.")
    

class NodeList:
    """
    This class implements data structures to store nodes and costs.
    
    """
    
    def __init__(self,capacity):
        # binary heap for storing costs
        self.__cost_memory = []
        # dictionary for storing nodes
        self._node_memory = {}
        self._length = 0
        self.capacity = capacity
        
    def isempty(self):
        return self.__length == 0
        
    def get(self,key):
        if key in self.node_memory.keys():
            return self.node_memory[key]
        return None
        
    def push(self,node):
        if self._length < self.capacity:
            fcost = node.computeFCost()
            dstate = node.dstate
            heapq.heappush(self.memory,(fcost,dstate))
            self.node_memory[dstate] = node
            self.__length += 1
        else:
            print("Unable to push item because the list is full")
        
    def pop(self):
        if self._length > 0:
            item = heapq.heappop(self.memory)
            dstate = item[1]
            node = self.node_memory[dstate]
            self._length -= 1
            return node
        return None
    
    def replace(self,current,new):
        dstate = current.dstate
        if dstate in self.node_memory.keys():
            self.node_memory[dstate] = new
        else:
            print("Unable to find a matching key to replace an item")
    
    

    