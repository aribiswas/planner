# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
from node import Node
import numpy as np
import graphics


class AbstractWorld(ABC):
    
    @property
    @abstractmethod
    def origin(self):
        pass
    
    @property
    @abstractmethod
    def limits(self):
        pass
    
    @property
    @abstractmethod
    def resolution(self):
        pass
    
    @property
    @abstractmethod
    def start_pose(self):
        pass
    
    @property
    @abstractmethod
    def goal_pose(self):
        pass 
    
    def __init__(self):
        self.vehicle = None
        self.obstacles = []
        super()
    
    @abstractmethod
    def render(self):
        pass
        
    @abstractmethod
    def computeHCost(self,current,goal):
        pass
        
    def addVehicle(self,vehicle):
        self.vehicle = vehicle
        
    def addObstacle(self,obstacle):
        self.obstacles.append(obstacle)
        
    def c2d(self,cstate):
        x = (cstate[0] - self.origin[0]) / self.resolution
        y = (cstate[1] - self.origin[1]) / self.resolution
        theta = np.round(cstate[2] / self.yaw_resolution) * self.yaw_resolution
        return np.array([x,y,theta])
    
    def createFuture(self,current_node):
        futures = []
        # create motion primitives at current node
        primitives = self.car.createMotionPrimitives(current_node.cstate)
        # create future nodes
        for p in primitives:
            # if the primitive does not have collisions then create new node
            if self.isValidPrimitive(p):
                cstate = p.end_pose
                dstate = self.c2d(cstate)
                gcost = current_node.gcost + p.getCost()
                hcost = self.computeHCost(cstate)
                path = p.getPath()
                node = Node(cstate,dstate,gcost,hcost,current_node,path)
                futures.append(node)
        return futures
    
    def isValidPrimitive(self,primitive):
        path = primitive.getPath()
        for pose in path:
            rect = self.vehicle.getFootPrint(pose)
            # check if rect collides with an obstacle
            for ob in self.obstacles:
                if ob.intersects(rect):
                    return False
        return True
        
        
        
        
class ParkingLot(AbstractWorld):
    
    def __init__(self,
                 start,
                 goal,
                 origin = (0,0),
                 limits = ((0,10),(0,10)),
                 xyres = 1,
                 yawres = 1):
        super()
        self.origin = origin
        self.limits = limits
        self.resolution = xyres
        self.yaw_resolution = yawres
        self.start_pose = start
        self.goal_pose = goal
        
    def plot(self):
        fig,ax = graphics.newPlot(self.limits)
        
        # plot start and end pose
        ax.plot(self.start_pose[0],self.start_pose[1],'ro')
        ax.plot(self.end_pose[0],self.end_pose[1],'go')
        
        # plot obstacles
        for ob in self.obstacles:
            x,y = ob.center
            angle = ob.angle
            graphics.newRectangle(ax,(x,y,angle),ob.length,ob.width)
            
        # plot vehicle
        vehicle_rect = None
        if self.vehicle is not None:
            x,y,angle = self.start_pose
            vehicle_rect = graphics.newRectangle(ax,(x,y,angle),ob.length,ob.width)
        
        return fig,ax,vehicle_rect
    
    def computeHCost(self,current,goal):
        """
        Return manhattan distance as the H cost

        """
        d = abs(current - goal)
        return sum(d)
    
    
        
        
        
        
    
    