# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def newPlot(limits):
    fig = plt.figure()
    fig.set_dpi(200)
    fig.set_size_inches(7, 6.5)
    ax = plt.axes(xlim=limits[0], ylim=limits[1])
    return fig,ax

def newRectangle(ax,pose,length,width):
    x,y,angle = pose
    rectangle = plt.Rectangle((x-0.5*length,y-0.5*width), length, width, fc='g')
    rectangle.angle = angle
    rectangle.center = (x,y)
    ax.add_patch(rectangle)
    return rectangle
        
def updateRectangle(frame,rectangle,path):
    pose = path[frame]
    x,y,angle = pose
    rectangle.angle = angle
    rectangle.center = (x,y)
    return rectangle
    
def animatePath(fig,vehicle_rect,path):
    frames = range(path.shape[0])
    FuncAnimation(fig, updateRectangle, frames, fargs=(vehicle_rect,path), interval=100)
        