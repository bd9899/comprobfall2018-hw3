# -*- coding: utf-8 -*-
"""
Created on Sat Dec  8 14:32:39 2018

@author: jackc
"""

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import uniform 
from filterpy.stats import plot_gaussian_pdf
import math
from shapely import geometry
from descartes.patch import PolygonPatch
import shapely.geometry.polygon as pg
import time
from shapely.geometry.point import Point
from shapely.geometry import LineString
import matplotlib.patches as ptc
import pandas as pd 
import random as rnd


worldBounds = [[-5,1],[-3,-2]]    #bounds of the world/room
Pi = math.pi
obstacles = []

class Node():
    pos = [0,0,0]
    x = 0
    y = 0
    r = 0
    connected = []
    parent = None
    g = 0
    
    def __str__(self):
        temp = 'Position: ' + '('+ str(self.x)
        temp += ', ' + str(self.y) + ')'
        return temp
    def setPos(self,x,y): #set position of Node at x,y,z
        self.x = x
        self.y = y
        self.pos[0] = x
        self.pos[1] = y
        return self.pos
    
    def setRot(self, angle):
        self.r = angle
        self.pos[2] = angle
        return self.pos

def makeNode(): #generates a random node with random Quaternion/position
    node = Node()
    global worldBounds
    bounds = worldBounds
    posX = rnd.random()*(bounds[0][1]-bounds[0][0]) + bounds[0][0] 
    posY = rnd.random()*(bounds[1][1]-bounds[1][0]) + bounds[1][0] 
    rotation = rnd.random()*(2*Pi)-Pi
    node.pos = [posX,posY, rotation]
    node.x = posX
    node.y = posY
    node.r = rotation
    
    return node



def func():
    N = 20000  # number of points
    radius = 1.
    area = (2*radius)**2
    
    pts = uniform(-1, 1, (N, 2))
    
    # distance from (0,0) 
    dist = np.linalg.norm(pts, axis=1)
    in_circle = dist <= 1
    
    pts_in_circle = np.count_nonzero(in_circle)
    pi = 4 * (pts_in_circle / N)
    
    # plot results
    plt.scatter(pts[in_circle,0], pts[in_circle,1], 
                marker=',', edgecolor='k', s=1)
    plt.scatter(pts[~in_circle,0], pts[~in_circle,1], 
                marker=',', edgecolor='r', s=1)
    plt.axis('equal')
    
    print 'mean pi(N={})= {:.4f}'.format(N, pi)
    print 'err  pi(N={})= {:.4f}'.format(N, np.pi-pi)
    
    fig, ax = plt.subplots(1,1)
    plot_gaussian_pdf(mean=2, variance=3);
    

def drawWorld(ax):
    minX = worldBounds[0][0]
    maxX = worldBounds[0][1]
    minY = worldBounds[1][0]
    maxY = worldBounds[1][1]
    ax.axis([minX - .5, maxX+.5, minY-.5, maxY+.5])
    for poly in obstacles:
#        print poly.exterior.coords.xy
        patch = PolygonPatch(poly, facecolor=[.3,0,0.5], edgecolor=[0,0,0], alpha=0.4, zorder=2)
        ax.add_patch(patch)

def makeWorld(fileName,k):
    file = open(fileName)
    lines = file.readlines()
    coords = lines[0].replace('(','').replace(')','').split()
    xVals = []
    yVals = []
    for xy in coords:
        xVals.append(xy.split(',')[0])
        yVals.append(xy.split(',')[1])
    
    minX = (float)(min(xVals))
    maxX = (float)(max(xVals))
    minY = (float)(min(yVals))
    maxY = (float)(max(yVals))
    global worldBounds
    worldBounds = [[minX, maxX],[minY, maxY]]
    lines = lines[2::] #delete first two lines
    
    global obstacles
    for i in range(k):
        vertices = []
        ln = lines[0]
        coords = ln.replace('(','').replace(')','').split()
        for xy in coords:
            xVal = (float)(xy.split(',')[0])
            yVal = (float)(xy.split(',')[1])
            vertices.append([xVal,yVal])
            
        obs = ptc.Polygon(vertices)
        poly = geometry.Polygon(obs.get_xy())
        obstacles.append(poly)
        lines = lines[1::] 






def main():
    
    makeWorld('grid1.txt',3)
    
    graph = True
    
    if graph:
        fig, ax = plt.subplots(1,1,figsize = (10,10))
        drawWorld(ax)
        print str(ax.axes)



if __name__ == '__main__':
    main()














