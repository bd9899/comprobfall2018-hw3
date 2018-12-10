#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Dec  8 19:06:32 2018

@author: root
"""
true_heading = []
true_distance = []
position = []
orientation = []
linear = []
angular = []
noisy_heading = []
noisy_distance = []
scan_data = []
start_pos = 0
std_dev = 0

def readFile(trajectory_filename = 'trajectories_1.txt'):
    
    f = open('turtlebot_maps/trajectories/' + trajectory_filename, 'r')
    while True:
        line = f.readline()
        if not line:
            break
        
        line = line.strip()
        a = line.split(' ')
        if a[0] == 'start:':
            global start_pos
            line = f.readline().strip()
            x = line.split(' ')[1]
            line = f.readline().strip()
            y = line.split(' ')[1]
            start_pos = (float(x),float(y))
        elif a[0] == 'Heading:':
            true_heading.append(float(a[2]))
        elif a[0] == 'Distance:':
            true_distance.append(float(a[2]))
        elif a[0] == 'position:':
            temp = []
            for i in range(3):
                line = f.readline().strip()
                b = line.split(' ')
                temp.append(float(b[1]))
            position.append(temp)
        elif a[0] == 'orientation:':
            temp = []
            for i in range(4):
                line = f.readline().strip()
                b = line.split(' ')
                temp.append(float(b[1]))    
            orientation.append(temp)
        elif a[0] == 'linear:':
            temp = []
            for i in range(3):
                line = f.readline().strip()
                b = line.split(' ')
                temp.append(float(b[1]))      
            linear.append(temp)
        elif a[0] == 'angular:':
            temp = []
            for i in range(3):
                line = f.readline().strip()
                b = line.split(' ')
                temp.append(float(b[1]))
            angular.append(temp)
        elif a[0] == 'noisy_heading:':
            line = f.readline().strip()
            b = line.split()
            noisy_heading.append(float(b[1]))
        elif a[0] == 'noisy_distance:':
            line = f.readline().strip()
            b = line.split()
            noisy_distance.append(float(b[1]))
        elif a[0] == 'scan_data:':
            line = f.readline().strip()
            b = line.strip("ranges: []")
            c = b.split(' ')
            t = []
            for x in c:
                x = x.strip(',')
               
                if x =='nan':
                    t.append(-1.0)
                elif x == '':
                    continue
                else:
                    t.append(float(x))
            scan_data.append(t)

def print_all():
    print '----------------------True Heading-----------------------------'
    print true_heading
    print len(true_heading)
    print '----------------------True Distance-----------------------------'
    print true_distance
    print len(true_distance)
    print '----------------------Noisy Distance-----------------------------'
    print noisy_distance
    print len(noisy_distance)
    print '----------------------Noisy Heading-----------------------------'
    print noisy_heading
    print len(noisy_distance)
    print '----------------------Angular-----------------------------'
    print angular
    print len(angular)
    print '----------------------Linear-----------------------------'
    print linear
    print len(angular)
    print '----------------------Position-----------------------------'
    print position
    print len(position)
    print '----------------------Orientation-----------------------------'
    print orientation
    print len(orientation)
    print '----------------------Start Pos-----------------------------'
    print start_pos
    print '----------------------Scan Data-----------------------------'
    print scan_data
    print len(scan_data)
        

#        

        