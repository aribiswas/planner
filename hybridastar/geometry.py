# -*- coding: utf-8 -*-

import numpy as np

class Rectangle:
    """
    Rectangle convention:
        
      D---------C   
      |    x    |
      A---------B
      
    """
    
    def __init__(self,center,length,width,angle=0):
        """
        Initialize a Rectangle.

        Parameters
        ----------
        center : tuple, numpy.ndarray
            (x,y) coordinates of the center of the rectangle.
        length : float
            Length of the rectangle along x direction.
        width : float
            Width of the rectangle along y direction.
        angle : float, optional
            Angle of rotation of the rectangle about the center. The default is 0.

        Returns
        -------
        None.

        """
        
        # validate arguments
        self._validate_args(center,length,width,angle)
        
        self.center = np.array(center)
        self.length = length
        self.width = width
        self.R = np.array([ [np.cos(angle),-np.sin(angle)],
                            [np.sin(angle),np.cos(angle)] ])
        
        # vertices in local coordinates
        self.A = 0.5 * np.array([-length,-width])
        self.B = 0.5 * np.array([length,-width])
        self.C = 0.5 * np.array([length,width])
        self.D = 0.5 * np.array([-length,width])
        
    def globalCoordinates(self):
        """
        Get the global coordinates of the vertices of the rectangle.

        Returns
        -------
        g : List of numpy.ndarray
            A,B,C,D coordinates in global frame.

        """
        # rotate vertices about local center
        rA = self.R @ self.A
        rB = self.R @ self.B
        rC = self.R @ self.C
        rD = self.R @ self.D
        
        # translate to global center
        g = []
        g.append(rA + self.center)
        g.append(rB + self.center)
        g.append(rC + self.center)
        g.append(rD + self.center)
        return g
                
    def contains(self,point):
        """
        Check if point lies within self.
        point is expressed in global coordinate system

        Parameters
        ----------
        point : tuple, numpy.ndarray
            (x,y) coordinates of the point.

        Returns
        -------
        bool
            True if point lies within the rectangle.

        """
        
        # transform the point to local coordinate system
        pself = self.R @ (point - self.center)
        
        # check if pself is inside the rectangle
        if pself[0]>=self.A[0] and pself[0]<=self.B[0] and pself[1]>=self.A[1] and pself[1]<=self.D[1]:
            return True
        
        return False
    
    def intersects(self,rect):
        """
        Check if rect overlaps with self.
        
        To check the intersection of two rectangles,
            1. For each vertex in rect, check if it lies within self
            2. For each vertex in self, check if it lies within rect

        """
        
        # check #1
        vertices = rect.globalCoordinates()
        for v in vertices:
            if self.contains(v):
                return True
        
        # check #2
        vertices = self.globalCoordinates()
        for v in vertices:
            if rect.contains(v):
                return True
            
        return False
    
    def _validate_args(self,center,length,width,angle):
        if not isinstance(center,(tuple,list,np.ndarray)):
            raise TypeError("center must be a 2 element tuple, list or numpy array.")
        if not isinstance(length,(int,float)):
            raise TypeError("length must be a scalar int or float.")
        if not isinstance(width,(int,float)):
            raise TypeError("width must be a scalar int or float.")
        if not isinstance(angle,(int,float)):
            raise TypeError("angle must be a scalar int or float.")
        if len(center)!=2:
            raise ValueError("center must be a 2 element tuple or numpy array.")
        if length<=0 or width<=0:
            raise ValueError("length and width must be positive.")
        