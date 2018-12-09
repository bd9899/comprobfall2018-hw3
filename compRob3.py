# -*- coding: utf-8 -*-
"""
Created on Sat Dec  8 14:32:39 2018

@author: jackc
"""

import matplotlib.pyplot as plt
import numpy as np
import os
from numpy.random import uniform 
from numpy.random import randn
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
particles = []
nodes = [] #tuple of (node, weight)


class Node():
    pos = [0,0,0]
    x = 0
    y = 0
    r = 0
    
    def __str__(self):
        temp = 'Position: ' + '('+ str(self.x)
        temp += ', ' + str(self.y) + ')' + ' Rotation: ' + str(self.r)
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

def makeNode(posX, posY, rotation): #generates a random node with random Quaternion/position
    node = Node()
#    global worldBounds
#    bounds = worldBounds
#    posX = rnd.random()*(bounds[0][1]-bounds[0][0]) + bounds[0][0] 
#    posY = rnd.random()*(bounds[1][1]-bounds[1][0]) + bounds[1][0] 
#    rotation = rnd.random()*(2*Pi)-Pi
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


def createUniform(xRange, yRange, rRange, N):
    particles = np.empty((N, 3))
    particles[:, 0] = uniform(xRange[0],xRange[1], size=N)
    particles[:, 1] = uniform(yRange[0],yRange[1], size=N)
    particles[:, 2] = uniform(rRange[0],rRange[1], size=N)
    particles[:, 2] %= 2 * Pi
    particles[:, 2] -= Pi #to make it range -Pi to Pi
    
    return particles

def createGaussian(meanVec, stdVec, N):
    particles = np.empty((N, 3))
    particles[:, 0] = meanVec[0] + (randn(N) * stdVec[0])
    particles[:, 1] = meanVec[1] + (randn(N) * stdVec[1])
    particles[:, 2] = meanVec[2] + (randn(N) * stdVec[2])
    particles[:, 2] %= 2 * Pi
    particles[:, 2] -= Pi
    
    return particles

def predict(particles, u, std, dt=1.):
    """ move according to control input u (heading change, velocity)
    with noise Q (std heading change, std velocity)`"""

    N = len(particles)
    # update heading
    particles[:, 2] += u[0] + (randn(N) * std[0])
    particles[:, 2] %= 2 * Pi
    particles[:, 2] -= Pi

    # move in the (noisy) commanded direction
    dist = (u[1] * dt) + (randn(N) * std[1])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist

    return particles

def update():
    a = 1
    
def propagate():
    a = 1
    







def main():
    
    makeWorld('grid1.txt',3)
    a = createGaussian([0,0,0], [2,2,2] ,5)
    
#    if os.name == 'nt':
#        clear = lambda: os.system('!CLS')
    print '\033[H\033[J'
#    else:
#        clear = lambda: os.system('!clear')
    
    
    
    
    
    
    plt.gcf().clear()
    
    graph = True
    if graph:
        fig, ax = plt.subplots(1,1,figsize = (10,10))
#        plt.gcf().clear()    
        time.sleep(.01)
        drawWorld(ax)
        ax.plot(rnd.uniform(-6, 4),rnd.uniform(-6, 4), 'ro', markersize = 5)
        plt.show()
#            clear()
        time.sleep(.01)
#        print str(ax.axes)



if __name__ == '__main__':
    main()














