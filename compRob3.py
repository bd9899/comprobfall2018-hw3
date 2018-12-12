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
from shapely.geometry.point import Point
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
import scipy


worldBounds = [[-5,1],[-3,-2]]    #bounds of the world/room
Pi = math.pi
obstacles = []
particles = []
weights = []
noise = [] #tuple of (scan noise, trans noise, rotation noise)
#Nodes = [] #tuple of (node, weight)
distribution = None
N = 100
NTh = N/2.0
INITIAL_HEADING = 0
iterParticles = []
iterReal = []

class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.z = theta
        

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
        
    
    worldEdgeList = []
    worldEdgeList.append([[minX-.5, minY-.5], [minX-.5, minY], [maxX + .5, minY], [maxX + .5, minY-.5], [minX-.5, minY-.5]])
    worldEdgeList.append([[minX-.5, maxY+.5], [minX-.5, maxY], [maxX + .5, maxY], [maxX + .5, maxY+.5], [minX-.5, maxY+.5]])
    worldEdgeList.append([[minX-.5, minY-.5], [minX-.5, maxY+.5], [minX, maxY+.5], [minX, minY-.5], [minX-.5, minY-.5]])
    worldEdgeList.append([[maxX+.5, minY-.5], [maxX+.5, maxY+.5], [maxX, maxY+.5], [maxX, minY-.5], [maxX+.5, minY-.5]])
    
    for vertices in worldEdgeList:
        obs = ptc.Polygon(vertices)
        poly = geometry.Polygon(obs.get_xy())
        obstacles.append(poly)
    
    return poly

def visualize(est, real):
    xEst = np.zeros((3, 1))
    xTrue = np.zeros((3, 1))

    i = 0

    while i < len(real):
        
        particle_list = est[i]   
        real1 = real[i]

        i+=1
        if True:
            fig, ax = plt.subplots(1,1,figsize = (10,10))
            drawWorld(ax)

            ax.plot(real1[0],real1[1], "bv", markersize = 5)
            for parts in particle_list:
#                print(parts)
                ax.plot(parts[0],parts[1], "ro", markersize = 2)
            
            ax.set_title('Iteration: ' + str(i-1))
            plt.show()
        

def createUniform(xRange, yRange, rRange, N):
    particles = []
    
    for i in range(N):
        xRand = rnd.uniform(xRange[0], xRange[1])
        yRand = rnd.uniform(yRange[0], yRange[1])
        rRand = rnd.uniform(rRange[0], rRange[1])
        particles.append([xRand, yRand, rRand])
    
    particles = np.array(particles)
    weights = np.empty((N,1))
    weights.fill(1.0/N)
    
    return particles, weights
    


def createGaussian(meanVec, stdVec, N):
    particles = np.empty((N, 3))
    particles[:, 0] = meanVec[0] + (randn(N) * stdVec[0])
    particles[:, 1] = meanVec[1] + (randn(N) * stdVec[1])
    particles[:, 2] = rnd.uniform(-.1,.05)#meanVec[2] + (randn(N) * stdVec[2])
    #particles[:, 2] %= 2 * Pi
    #particles[:, 2] -= Pi
    weights = np.empty((N,1))
    weights.fill(1.0/N)

    return particles, weights



def predict(u):

    global particles
    global noise
    
    N = len(particles)
    # update heading
    
    particles[:, 2] = np.random.normal(u[1],noise[2])
#    particles[:,2] %= 2*Pi
#    normalizeAngles()

    dist = np.random.normal(u[0], noise[1])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist
    
    return particles

def normalizeAngles():
    global particles
    
    for i in range(len(particles)):
        while particles[i,2] > Pi:
            particles[i,2] -= 2*Pi
        while particles[i,2] < -Pi:
            particles[i,2] += 2*Pi


def likelihood(distance, measured, noise = .01): #measured distance 
    
    prob = scipy.stats.norm(measured, noise).pdf(distance)
    
    return prob
    

def updateWeights(measuredLengths):
    global weights, particles
    
    for i in range(len(weights)):
        x = particles[i,0]
        y = particles[i,1]
        z = particles[i,2]

        scans = generate_scans_for_particles(Pose(x,y,z))

        for j in range(len(scans)):
            if measuredLengths[j] == -1.0:
                continue
            prob = likelihood(scans[j], measuredLengths[j], noise = noise[0])

            weights[i] *= prob
        if x > worldBounds[0][1] or x < worldBounds[0][0]:
            weights[i] = -1.e-301
        if y > worldBounds[1][1] or y < worldBounds[1][0]:
            weights[i] = -1.e-301
        for poly in obstacles:
            if poly.contains(Point(x,y)):
                weights[i] = -1.e-301
                break
        weights[i] += 1.e-300      # avoid round-off to zero
    
    totalWeight = sum(weights)
    for i in range(len(weights)):
        weights[i] /= totalWeight # normalize 
    
    return weights
    
    

def resample_from_index(indexes):
    global particles, weights
    
    particles[:] = particles[indexes]
#    weights[:] = weights[indexes]
    weights.fill(1.0 / len(weights))
    
    particles = particles.round(5)
    meanVec = [np.mean(particles[:,0]),np.mean(particles[:,1]),np.mean(particles[:,2])]
    stdVec = [np.std(particles[:,0]),np.std(particles[:,1]),np.std(particles[:,2])]    
    
    uniqueParticles = np.unique(particles, axis=0)
    
    tempNum = int(N/7/len(uniqueParticles))
    numAdded = int(tempNum*len(uniqueParticles))
    if numAdded <= N/6 and tempNum > 0:
        newParts, w = createGaussian(uniqueParticles[0], [.1,.1,.1] , tempNum)
        for i in range(1,len(uniqueParticles)):
            tempNew, w = createGaussian(uniqueParticles[0], [.1,.1,Pi/40] , tempNum)
            newParts = np.concatenate((newParts, tempNew))
    
        inds = np.random.randint(0,N,numAdded)
        newParts, w = createGaussian(meanVec, stdVec, numAdded)
        particles[inds] = newParts
    
    
    return particles

def systematic_resample(weights):
    N = len(weights)

    # make N subdivisions, and choose positions with a consistent random offset
    positions = (rnd.random() + np.arange(N)) / N

    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    
    i,j = 0,0
    while i < N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes

def resample2():
    global weights, particles

    neff =  1. / np.sum(np.square(weights))
#    neff = len(np.unique(particles, axis=0))
    if neff < NTh:
        print 'neff     ', neff
        indexes = systematic_resample(weights)
        resample_from_index(indexes)
#        assert np.allclose(weights, 1/N)  




def grabTop5():
    global weights
    
    sort_list = weights.copy()
    sort_list = np.argsort(sort_list)
    return sort_list[:6]

def takeDistribution(mu, signma):
    global N
    return np.random.normal(mu, signma, size=N/5).reshape((N/5,1))
    
def resample():
    global weights, particles
    unique_particles = set()
    org_weights = []
    
    for i in range(len(weights)):
        org_weights.append(float(weights[i]))
        
    for i in range(len(org_weights)):
        unique_particles.add(org_weights[i])
        
    if len(unique_particles) < N/10:
        top5 = grabTop5()
        resample_particles = []
        for values in range(len(top5)):
            
            pose = particles[top5[values],:]
            print(pose)
            X = takeDistribution(float(pose[0][0]),.2)
            Y = takeDistribution(float(pose[0][1]),.2)
            Z = takeDistribution(float(pose[0][2]),Pi/10)
            temp = np.concatenate((X,Y,Z), axis=1)
            if len(resample_particles) == 0:
                    resample_particles = temp.copy()
            else:
                resample_particles = np.concatenate((resample_particles, temp ))
        particles = resample_particles
        particles = np.array(particles)

    return particles


   
    
def resample3():
    global weights, particles, NTh
    Neff =  1. / np.sum(np.square(weights))
    if Neff < NTh:
        
        wcum = np.cumsum(weights)
        base = np.cumsum(np.array(weights) * 0.0 + 1 / N) - 1 / N
        resampleid = base + np.random.rand(base.shape[0]) / N
    
        inds = []
        ind = 0
        for ip in range(N):
            while resampleid[ip] > wcum[ind]:
                ind += 1
            inds.append(ind)
    
        particles = particles[inds, :]
        weights = [1.0 / N for i in range(N)] # init weight


def compute_x_y(pose):
    x = pose.x
    y = pose.y
    theta = pose.z
    
    new_y = math.sin(theta)*10
    new_x = math.cos(theta)*10
    
    return Pose(new_x, new_y, theta)
    
    
def generate_scans_for_particles(pose):
        
        global obstacles
        
        theta_start = pose.z- 0.523599
        theta_stop = pose.z + 0.523599
        STEP_SIZE = 0.019652407
        
        steps = np.arange(theta_start, theta_stop, STEP_SIZE)
        steps = steps[::-1]
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



def particleFilter(iterations, isStartKnown = False, graph = False):
    global particles, weights, iterParticles, iterReal, N
    
    if isStartKnown:
        meanVec = [rd.start_pos[0],rd.start_pos[1],INITIAL_HEADING]
        stdVec = [.3, .3, Pi/20]
        particles, weights = createGaussian(meanVec, stdVec, N)
    else:
        particles, weights = createUniform(worldBounds[0], worldBounds[1], [-Pi, Pi], N)

    particles = particles.round(5)

    prevHeading = INITIAL_HEADING   
    
    startPos = np.array([rd.start_pos[0],rd.start_pos[1], INITIAL_HEADING])
    iterParticles.append(particles.copy())
    iterReal.append(startPos)
    
    changeList = []
    
    print 'start'
    for i in range(iterations):        
        print 'iterations', i
        sTime = time.time()
        
        angleChange = rd.noisy_heading[i] - prevHeading
        prevHeading = rd.noisy_heading[i]
        distanceChange = rd.noisy_distance[i]
        
        changeList.append([distanceChange, angleChange])
#        predict([distanceChange, angleChange])
        predict([distanceChange, rd.noisy_heading[i]])
        particles = particles.round(5)
        updateWeights(rd.scan_data[i])
        resample2()
        
        realPos = np.array(rd.position[i])
        iterParticles.append(particles.copy())
        iterReal.append(realPos)
        
        print time.time() - sTime
        print len(iterParticles[i+1])
        print 'num uninque ', len(np.unique(iterParticles[i+1], axis=0))
        
    if graph:
        visualize(iterParticles, iterReal)
    
    print iterParticles[i+1]
    print np.unique(iterParticles[-1], axis=0)
#    for q in range(1,len(iterParticles)):
#        print changeList[q-1]
#        xVal = iterParticles[q][0][0]
#        yVal = iterParticles[q][0][1]
#        xValP = iterParticles[q-1][0][0]
#        yValP = iterParticles[q-1][0][1]
#        diffDist = ((yVal-yValP)**2 + (xVal-xValP)**2)**.5
#        diffAngle = iterParticles[q][0][2] - iterParticles[q-1][0][2]
#        print 'difference', diffDist, ' , ' , diffAngle 


    return particles, weights   


def main(scan_n = 0.1, trans_n = 0.1, rot_n = .1):
    global INITIAL_HEADING, N
    global noise
    
    noise = (scan_n, trans_n, rot_n)
    INITIAL_HEADING = 0    
    
    
    
    rd.readFile('trajectories_1.txt')
    makeWorld('grid1.txt',3)
    
    N = 100
    print 'total iterations', len(rd.position), ' with ', N, ' number of particles'
    particleFilter(len(rd.position), isStartKnown = False, graph =True)
    



if __name__ == '__main__':
    main()














