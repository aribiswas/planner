# -*- coding: utf-8 -*-

import numpy as np
from abc import ABC, abstractmethod
from geometry import Rectangle  


class AbstractVehicle(ABC):
    
    @property
    @abstractmethod
    def dimensions(self):
        pass
    
    @property
    @abstractmethod
    def wheel_base(self):
        pass
    
    @property
    @abstractmethod
    def sample_time(self):
        pass
    
    @property
    @abstractmethod
    def steering_resolution(self):
        pass
    
    @property
    @abstractmethod
    def steering_limit(self):
        pass
    
    @property
    @abstractmethod
    def motion_primitive_length(self):
        pass
    
    def __init__(self,params):
        self.__computeBasePrimitives()
        super()
        
    @abstractmethod
    def step(self,state,action):
        pass
    
    def getFootPrint(self,state):
        center = state[0:1]
        length = self.dimensions[0]
        width = self.dimensions[1]
        angle = state[2]
        return Rectangle(center,length,width,angle)
        
    def __computeBasePrimitives(self):
        self.__base_primitives = []
        num_primitives = np.floor(self.steering_limit/self.steering_resolution)
        steering_angles = self.steering_limit * np.linspace(-1,1,num=num_primitives)
        for action in steering_angles:
            state = np.array([0,0,0])
            primitive = []
            d = 0
            while d <= self.motion_primitive_length:
                # log the state
                primitive.append(state)                
                # compute the next state
                next_state = self.step(state,action)
                # increase d by distance traversed
                d += np.sqrt(np.sum((next_state[:1]-state[:1])**2))
                # update the state
                state = next_state
            # log the motion primitive
            self.__base_primitives.append(np.array(primitive))
    
    def createMotionPrimitives(self,state):
        primitives = []
        X,Y,THETA = [item for item in state]
        c,s = np.cos(THETA), np.sin(THETA)
        rotation_matrix = np.array([c,-s],[s,c])
        for path in self.__base_primitives:
            path_xy = rotation_matrix @ path[:,[0,1]].T
            primitives.append(path_xy.T)
            
            
def Car(AbstractVehicle):

    def __init__(self,
                 dimensions = (4.7,1.8),
                 wheelbase = 2.8,
                 steering_res = 10*np.pi/180,
                 steering_limit = np.pi/4,
                 primitive_length = 2,
                 sample_time = 0.1):
        self.dimensions = dimensions
        self.wheel_base = wheelbase
        self.steering_resolution = steering_res
        self.steering_limit = steering_limit
        self.motion_primitive_length = primitive_length
        self.sample_time = sample_time
        super()
        
    def step(self,state,action):
        """
        Step the vehicle to the next state given the current state and action.
        
        """
        # RK4 approximation
        x = state
        u = action
        h = self.sample_time
        k1 = h * self.dynamics(x,u)
        k2 = h * self.dynamics(x+k1/2,u)
        k3 = h * self.dynamics(x+k2/2,u)
        k4 = h * self.dynamics(x+k3,u)
        next_state = x + 1/6 * (k1 + 2*k2 + 2*k3 + k4)
        
        return next_state
        
    
    def dynamics(self,q,u):
        """
        Single track bicycle dynamics model
        
        """
        theta  = q[2]
        v      = u[0]
        d      = u[1]
        L      = self.wheel_base
        xd     = v * np.cos(theta)
        yd     = v * np.sin(theta)
        thetad = abs(v/L) * np.tan(d)
        
        return np.array([xd,yd,thetad])

        
                
