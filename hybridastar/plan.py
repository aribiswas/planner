# -*- coding: utf-8 -*-

from node import Node, NodeList
import graphics

class Planner:
    
    def __init__(self,world,visuals=True):
        self.open = NodeList(capacity=10000)
        self.closed = NodeList(capacity=10000)
        self.world = world
        self.fig = None
        self.ax = None
        self.vehicle_rect = None
        self.viz = visuals
        
    def _updateNeighbors(self,current):
        
        # create futures from current node
        futures = self.world.createFuture(current)
        
        for node in futures:
            
            # skip if it is a closed node
            if self.closed.get(node.dstate) is not None:
                continue
            
            # draw transition path
            if self.viz:
                node.drawPath(self.ax)
            
            # replace existing open node if it has higher fcost than current
            match = self.open.get(node.dstate)
            if match is not None and node.computeFCost() < match.computeFCost():
                self.open.replace(match,node)
            else:
                self.open.push(node)
        
    def run(self,start_pose,goal_pose):
        
        # initialize graphics objects
        if self.viz:
            self.fig, self.ax, self.vehicle_rect = self.world.plot()
        
        # calculate initial cost of start pose
        start_gcost = 0
        start_hcost = self.world.computeHCost(start_pose,goal_pose)
        
        # create start node and add to open list
        node = Node(start_pose,start_gcost,start_hcost)
        self.open.push(node)
        
        # search for the goal
        path = None
        while self.open.isempty()==False:
            # pop from the open list and add to the closed list
            node = self.open.pop()
            self.closed.push(node)
            
            # check if goal is reached
            if self.goalReached(node.cstate):
                # trace the path to the start pose 
                path = node.trace()
                break
            else:
                # expand and update neighboring nodes
                self._updateNeighbors(node)
        
        # create an animation of the path
        if self.viz:
            self.animate()
        
        return path
    
    def animate(self,path):
        if path is not None:
            graphics.animatePath(self.fig,self.vehicle_rect,path)
    
    
    
        
            