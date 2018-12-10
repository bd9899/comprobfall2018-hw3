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
from shapely.geometry.polygon import LinearRing
from shapely.geometry.point import Point
from shapely.geometry import LineString
import matplotlib.patches as ptc
import pandas as pd 
import random as rnd
import read_file as rd


worldBounds = [[-5,1],[-3,-2]]    #bounds of the world/room
Pi = math.pi
obstacles = []
particles = []
weights = []
#Nodes = [] #tuple of (node, weight)
distribution = None
N = 100
NTh = N/2
INITIAL_HEADING = Pi/2

class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.z = theta
        

"""

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
  """  

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
    return poly

def visualize(est, real):
    xEst = np.zeros((3, 1))
    xTrue = np.zeros((3, 1))
    
    hxEst = xEst
    hxTrue = xTrue
    time = 0
    i = 0
    while 50 >= time and i < len(real):
        time += 0.01
        print(hxEst)
        
        est1 = np.array(est[i])
        est1 = est1.reshape((3,1))
        print(est1)
        
        real1 = np.array(real[i])
        real1 = real1.reshape((3,1))
        print(real1)

        hxEst = np.hstack((hxEst, est1))
        hxTrue = np.hstack((hxTrue, real1))
        i+=1
        if True:
            plt.cla()

            print(hxTrue)
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")

            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.show()
        

def createUniform(xRange, yRange, rRange):
    global particles, weights
    particles = np.empty((N, 3))
    particles[:, 0] = uniform(xRange[0],xRange[1], size=N)
    particles[:, 1] = uniform(yRange[0],yRange[1], size=N)
    particles[:, 2] = uniform(rRange[0],rRange[1], size=N)
    particles[:, 2] %= 2 * Pi
    particles[:, 2] -= Pi #to make it range -Pi to Pi
    weights = np.empty((N,1))
    weights.fill(1.0/N)
    return particles

def createGaussian(meanVec, stdVec):
    global particles, weights
    particles = np.empty((N, 3))
    particles[:, 0] = meanVec[0] + (randn(N) * stdVec[0])
    particles[:, 1] = meanVec[1] + (randn(N) * stdVec[1])
    particles[:, 2] = meanVec[2] + (randn(N) * stdVec[2])
    particles[:, 2] %= 2 * Pi
    particles[:, 2] -= Pi
    weights = np.empty((N,1))
    weights.fill(1.0/N)

    return particles
    
def init_particles():
    global particles
    N = len(particles)
    for particle in particles:
        particle = 1.0/N
        
    return 


def predict(u, std):
    """ move according to control input u (heading change, velocity)
    with noise Q (std heading change, std velocity)`"""
    global particles
    
    N = len(particles)
    # update heading
    particles[:, 2] += np.random.normal(u[1],std[1])
    particles[:, 2] %= 2 * Pi

    # move in the (noisy) commanded direction
    dist = np.random.normal(u[0],std[0])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist
    
    return particles


def likelihood(distance, measured, noise = 1): #measured distance 
    
    distance = distance - measured
    prob = 1/(2*Pi)**0.5
    prob /= noise
    prob *= -1* distance**2 /(2*noise**2)
    return prob
    

def updateWeights(measuredLengths):
    global weights, particles
    
    for i in range(len(weights)):
        x = particles[i,0]
        y = particles[i,1]
        z = particles[i,2]
        scans = generate_scans_for_particles(Pose(x,y,z))
#        print 'measured', len(measuredLengths)
#        print 'scans', len(scans)
        for j in range(len(scans)):
            if measuredLengths[j] == -1.0:
                continue
            prob = likelihood(scans[j], measuredLengths[j])
            weights[i] *= prob
        weights[i] += 1.e-300      # avoid round-off to zero
    
    for i in range(len(weights)):
        weights[i] /= sum(weights) # normalize 
    
    return weights
    
    

def resample_from_index(indexes):
    global particles, weights
    
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights.fill(1.0 / len(weights))
    
    return particles

def systematic_resample(weights):
    N = len(weights)

    # make N subdivisions, choose positions 
    # with a consistent random offset
    randNum = rnd.random()
    positions = (np.arange(N) + randNum) / N

    indexes = np.zeros(N, 'i')
    weightSum = np.cumsum(weights)
    i, j = 0, 0
    while i < N:
        if i == 100:
            print 'i'
        if j == 100:
            print 'j'
        if positions[i] < weightSum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes


def resample():
    global weights, particles
   
    neff =  1. / np.sum(np.square(weights))
    if neff < NTh:
        indexes = systematic_resample(weights)
        resample_from_index(indexes)
#        assert np.allclose(weights, 1/N)


    return particles


def particleFilter(iterations, graph = False):
    global particles, weights
    createUniform(worldBounds[0], worldBounds[1], [-Pi, Pi])
    prevHeading = INITIAL_HEADING
    if graph:
        startPos = np.array([rd.start_pos[0],rd.start_pos[1], INITIAL_HEADING])
        visualize(particles, startPos)
        
    for i in range(iterations):        
        print 'iterations', i

#        print 'heading'
#        print rd.noisy_heading[i]
#        print 'distance'
#        print rd.noisy_distance[i]
#        print
#        print rd.scan_data[i]
#        print 'done'
        
        ##counter clockwise spin is positive
        
        angleChange = rd.noisy_heading[i] - prevHeading
        prevHeading = rd.noisy_heading[i]
        distanceChange = rd.noisy_distance[i]
        noise = [.0000001, .0000001]
        
        predict([distanceChange, angleChange], noise)
        updateWeights(rd.scan_data[i])
        resample()
        
        if graph:
            realPos = np.array([rd.position[i]])
            visualize(particles, realPos)
    return particles, weights        
    

def compute_x_y(pose):
    x = pose.x
    y = pose.y
    theta = pose.z
    
    new_y = 10
    new_x = math.tan(theta)*new_y
    
    return Pose(new_x, new_y, theta)
    
    
def generate_scans_for_particles(pose):
        
        global obstacles
        
        theta_start = pose.z- 0.523599
        theta_stop = pose.z + 0.523599
        STEP_SIZE = 0.019652407
        
        steps = np.arange(theta_start, theta_stop, STEP_SIZE)
#        print(len(steps))
        scan = []
        distance = []
        
        for i in range(54):
            x = pose.x
            y = pose.y
            theta = steps[i]
            pose_temp = Pose(x,y,theta)
            pose_temp = compute_x_y(pose_temp)
            laser = (pose_temp.x+x, pose_temp.y+y)
            robot_pose = (pose.x, pose.y)
            possible_scans = []

            line = LineString([robot_pose, laser])
            for obstacle in obstacles:            
                polygons = LinearRing(list(obstacle.exterior.coords))
                intersections = polygons.intersection(line)
                if intersections:
                    if intersections.type == 'Point':
                        possible_scans.append((intersections.x,intersections.y))
                    else:
                        for points in intersections:
                            if points:
                                possible_scans.append((points.x,points.y))
                            
            x = compute_length(pose,possible_scans)
            #print(x)
            scan.append(x[0])
            distance.append(x[1])
            
        return distance
            
            


def compute_length(pose, possible_scans):
    
    lengths = []
    min_ = 100
    min_index = 0
    dist = 0
    for index,i in enumerate(possible_scans):
        x = i[0]
        y = i[1]
        dist = math.sqrt((pose.x-x)**2 + (pose.y-y)**2)
        if dist < min_:
            min_ = dist
            min_index = x,y
    if min_ == 100:
        min_ = -1
    if min_index == 0:
        min_index = 0,0
    return (min_index, min_)


def main():
    global INITIAL_HEADING
    
    INITIAL_HEADING = Pi/2
    pose = Pose(-7,-1.5,0)
    rd.readFile('trajectories_1.txt')
#    rd.print_all()
    makeWorld('grid1.txt',3)
    print 'total iterations', rd.position
    particleFilter(len(rd.position), graph =True)
    
    
    
    plt.gcf().clear()
    
    graph = True
    if graph:
        fig, ax = plt.subplots(1,1,figsize = (10,10))
        time.sleep(.01)
        drawWorld(ax)
        ax.plot(pose.x,pose.y,'ro', markersize=5)
        
        
        plt.show()
    
        time.sleep(.01)



if __name__ == '__main__':
    main()














