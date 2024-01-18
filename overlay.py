#!/usr/bin/env python

#Ashwin Subramanian, subraash@oregonstate.edu
#Try and localize robot within known map, starting position not given

import rosbag
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from PIL import Image
import pandas as pd
import random
import time
import os
from pathlib import Path

#s
from skimage.morphology import skeletonize
from scipy.ndimage import distance_transform_bf, distance_transform_edt
from skimage.morphology import medial_axis
from scipy.ndimage import morphology
from skimage import data, color, io
import matplotlib.pyplot as plt
from skimage.util import invert

#tensorflow
import cv2

class Particle():
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        self.color = np.random.randint(0, 10)
        self.path = [(x, y)]

class MapInterpolation():

    def __init__(self):
        
        #Bag Params
        self.dir = 'data/warehouse/'
        self.data_path = self.dir + '3/'
        self.bag_name = self.data_path + 'warehouse'
        Path(self.dir).mkdir(parents=True, exist_ok=True)
        Path(self.data_path).mkdir(parents=True, exist_ok=True)

        #Map Params
        self.map_width = 480
        self.map_height = 480
        self.resolution = 0.05
        self.origin_x = -12.0 
        self.origin_y = -12.0

        #Trial Params
        self.trials = 1
        self.variations = 1

        ####Display Options 
        self.visualize = False        #output images
        self.only_start_viz = False   #starting points vs. full paths
        self.genAlt = False           #compute alternative paths
        self.showOrgPath = False      #display original path

        self.paths = []
        self.path_coords = []
        self.map_data = []
        self.map_coords = []
        self.mcl_coords = []
        self.diff_vectors = []
        self.precise_diff_vectors = []

        self.map_array = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        self.bag = rosbag.Bag(self.bag_name + '.bag')
        self.viable = set()

        self.readMapData()
        self.binary_array = np.where(self.map_data == 0, 0, 1)

        self.readBag()
        self.diff_vectors = self.generateVectors(self.path_coords)
        # self.generatePreciseVectors()

        if self.genAlt:
            self.original_start = self.map_data[0]
            start_points = self.random_start(self.trials)

            for start_point in start_points:
                self.randomPath(start_point, self.variations)
        
        if self.showOrgPath:
            for (x, y) in self.map_coords:
                self.map_array[x, y] = 1

        if self.visualize:
            self.generateVisuals()

        # print("\nViable start positions: " + str(len(self.viable)) + ", " + str((len(self.viable)/(self.trials)) * 100) + "%")

    def random_start(self, trials):
        res = []
        for _ in range(trials):
            r_x = int(np.random.uniform(0, self.map_width))
            r_y = int(np.random.uniform(0, self.map_height)) 

            res.append((r_x, r_y))

        return res

    def collision(self, path):

        array = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        for x, y in path:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                array[y, x] = 1
            else:
                return False
            
        logical_and = np.logical_and(array, self.binary_array)
        points = np.argwhere(logical_and == True)

        if points.size == 0:
            return True

        return False

    def readBag(self):
        for topic, msg, t in self.bag.read_messages(topics='/odom'):
            pos = msg.__getattribute__('pose').__getattribute__('pose').__getattribute__('position')
            x = pos.__getattribute__('x')
            y = pos.__getattribute__('y')
            self.path_coords.append((y, -x)) #for map
            self.mcl_coords.append((-x, y)) #for mcl

    def generateVectors(self, coordinates):

        coords = coordinates

        #original points
        map_coords = []
        for x, y in coords:
            map_x = int((x - self.origin_x) / self.resolution)  
            map_y = int((y - self.origin_y) / self.resolution)
            map_coords.append((map_x, map_y))

        self.map_coords = list(dict.fromkeys(map_coords))

        #vectors
        diff_vectors = []
        for x in range(len(self.map_coords) - 1):
            vector = np.array([self.map_coords[x+1][0] - self.map_coords[x][0], self.map_coords[x+1][1] - self.map_coords[x][1]])
            mag = np.linalg.norm(vector)
            ang = np.arctan2(vector[1], vector[0])
            diff_vectors.append((mag, ang))

        return diff_vectors
        
    def generatePreciseVectors(self):
        adjusted_coords = []
        for x, y in self.path_coords:
            adjusted_coords.append((x / self.resolution, y / self.resolution))

        for x in range(len(adjusted_coords) - 1):
            vector = np.array([adjusted_coords[x+1][0] - adjusted_coords[x][0], adjusted_coords[x+1][1] - adjusted_coords[x][1]])
            mag = np.linalg.norm(vector)
            ang = np.arctan2(vector[1], vector[0])
            self.precise_diff_vectors.append((mag, ang))
        
    
    def randomPath(self, start, num_paths):
        for i in range(num_paths):
            path = [start]
            random_rotation = np.random.uniform(0, 360)
            for mag, ang in self.diff_vectors:
                r_mag = np.random.uniform(0.5, 2)
                r_ang = np.random.uniform(-np.radians(10), np.radians(10))

                adjusted_mag = mag * r_mag
                adjusted_ang = ang + r_ang + random_rotation

                r_x = path[-1][0] + int(adjusted_mag * np.cos(adjusted_ang))
                r_y = path[-1][1] + int(adjusted_mag * np.sin(adjusted_ang))
                
                path.append((r_x, r_y))

            if self.collision(path): #find a more effecient way to do this, this loop does not need to be run twice
                self.viable.add((start, random_rotation))
                if self.only_start_viz:
                    self.map_array[start[1], start[0]] = i+1
                else:
                    for x, y in path:
                        if 0 <= x < self.map_width and 0 <= y < self.map_height:
                            self.map_array[y, x] = i+1

    def readMapDataCallback(self, data):
        # self.map_info = data.info
        self.map_data = np.array(data.data, dtype=np.int8).reshape(self.map_width, self.map_height)
        
        # df = pd.DataFrame(self.map_data)
        # df.to_csv('map_data.csv', index=False)

    def readMapData(self):
        # rospy.init_node('map_subscriber')

        # rospy.Subscriber('/map', OccupancyGrid, self.readMapDataCallback)
        # rospy.spin()
        
        self.map_data = pd.read_csv(self.dir + 'map_data.csv').to_numpy()
        self.map_data = self.map_data[:, ::-1]
    
    def generateVisuals(self):

        colors = {
            0: (255, 255, 255), # White
            1: (255, 0, 0),     # Red
            2: (0, 255, 0),     # Lime
            3: (0, 0, 255),     # Blue
            4: (255, 255, 0),   # Yellow
            5: (255, 0, 255),   # Magenta
            6: (0, 255, 255),   # Cyan
            7: (255, 165, 0),   # Orange
            8: (128, 0, 128),   # Purple
            9: (0, 128, 128),   # Teal
            10: (128, 128, 0)   # Olive
        }

        colors_binary = { 0: (255, 255, 255),  
          1: (0, 0, 255)
          }
        
        mim = Image.new('RGB', (self.map_width, self.map_height))
        
        #path
        for x in range(self.map_width):
            for y in range(self.map_height):
                clr = colors[self.map_array[y, x]]
                mim.putpixel((x, y), clr)

        #map
        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.binary_array[y, x]:
                    clr = colors_binary[self.binary_array[y, x]]
                    mim.putpixel((x, y), clr)
        
        mim.save(self.bag_name + '.jpeg')

class MonteCarlo():

    def __init__(self, diff_vectors, obstacles, width, height, data_path):
        self.particles = []
        self.diff_vectors = diff_vectors
        self.obstacles = obstacles
        self.width = width
        self.height = height
        self.gif = []
        self.data_path = data_path

        #Params
        self.num_particles = 1000   #atleast 100000 to get a somewhat accurate result given a large map

        #Display Options
        self.showSteps = True          #This will generate a new Image every iteration vs. at the end. Keep image 
        self.start_point_viz = False    #Starting points only
        self.makeGif = True
        self.savePhoto = True

    def set_particles(self):
        for _ in range(self.num_particles):
            x = int(np.random.uniform(0, self.width))
            y = int(np.random.uniform(0, self.height))
            # theta = np.random.uniform(-np.radians(2 * np.pi), np.radians(2 * np.pi))
            theta = np.random.uniform(0, 360)
            weight = self.distance_transform[y, x]
            self.particles.append(Particle(x, y, theta, weight))

    def run_mcl(self):
        prev_diff = []
        self.gif = []
        for i, (mag, ang) in enumerate(self.diff_vectors):
            prev_diff.append((mag, ang))

            self.particles = self.motion_update(mag, ang, self.particles)
            self.particles = self.collision_update(self.particles)

            self.resample(self.num_particles - len(self.particles), prev_diff)

            if not self.particles : return
            if self.showSteps: self.print_map()
            print("Progress: " + str((i / len(self.diff_vectors)) * 100) + "%")

        self.print_map()
        if self.makeGif: self.createGif(self.gif)

    def resample(self, sampleSize, prev_diff):
        
        if sampleSize <= 0: return

        ###Set Particle
        particles = []
        weighted_particles = [1/particle.weight for particle in self.particles]
        for _ in range(sampleSize):
            # point = random.choice(self.particles)
            point = random.choices(self.particles, weights=weighted_particles, k=1)[0]

            x = int(point.x + np.random.uniform(-0.5, 0.5))
            y = int(point.y + np.random.uniform(-0.5, 0.5))
            theta = point.theta + np.random.uniform(-np.radians(2 * np.pi), np.radians(2 * np.pi))
            weight = self.distance_transform[y, x]

            particles.append(Particle(x, y, theta, weight))
        
        for mag, ang in prev_diff:
            particles = self.motion_update(mag, ang, particles)
            particles = self.collision_update(particles)

        for p in particles:
            self.particles.append(p)

        self.resample(sampleSize - len(particles), prev_diff)
    
            
    def motion_update(self, mag, ang, old_particles):
        particles = old_particles
        for i in range(len(particles)):
            p = particles[i]
            x = p.path[-1][0]
            y = p.path[-1][1]

            r_mag = np.random.uniform(0.9, 1.1)
            r_ang = np.random.uniform(-np.radians(20), np.radians(20))

            adjusted_mag = mag * r_mag
            adjusted_ang = ang + r_ang + p.theta

            # adjusted_mag = mag
            # adjusted_ang = ang 

            x = x + adjusted_mag * np.cos(adjusted_ang)
            y = y + adjusted_mag * np.sin(adjusted_ang)

            particles[i].path.append((x, y))
            
        return particles
        
    def collision_update(self, particles):
        t_particles = []
        for particle in particles:
            x = int(particle.path[-1][0])
            y = int(particle.path[-1][1])

            if 0 <= x < self.width and 0 <= y < self.height \
            and self.obstacles[y, x] == 0:
                particle.weight += self.distance_transform[y, x]
                t_particles.append(particle)

        return t_particles
        # self.num_particles = len(particles)
    
    # def wall_distance(self, particles):
    #     t_particles = []
    #     for particle in particles:
    #         x,y = particle.path[-1]


    #     return t_particles

    def medial_axis_weight(self):
        # binary_dilation = morphology.binary_dilation(self.obstacles, structure=np.ones((20,20))).astype(np.int64)

        # bt = (binary_dilation * 255).astype(np.uint8)

        image = self.obstacles == 1
        image = invert(image)


        image = np.ascontiguousarray(image, dtype=np.uint8)

        skeleton = skeletonize(image)

        combined = np.logical_or(mc.obstacles, np.where(skeleton == 1, 2, skeleton))

        og = Image.fromarray((self.obstacles).astype(np.uint8))
        og.show()
        im = Image.fromarray(skeleton)
        im.show()
        ca = Image.fromarray(combined)
        ca.show()

        self.distance_transform = distance_transform_edt(1 - skeleton.astype(np.int64))
        # for row in distance_transform:
        #     print(row)
        # c = Image.fromarray(distance_transform)
        # c.show()

    def print_map(self):
        mix = Image.new('RGB', (self.width, self.height), color='white')

        colors = {
            0: (255, 255, 255), # White
            1: (255, 0, 0),     # Red
            2: (0, 255, 0),     # Lime
            3: (255, 182, 193), # Pink
            4: (255, 255, 0),   # Yellow
            5: (255, 0, 255),   # Magenta
            6: (0, 255, 255),   # Cyan
            7: (255, 165, 0),   # Orange
            8: (128, 0, 128),   # Purple
            9: (0, 128, 128),   # Teal
        }

        colors_binary = { 0: (255, 255, 255),  
          1: (0, 0, 255)
          }
        
        #map
        for x in range(self.width):
            for y in range(self.height):
                if self.obstacles[y, x]:
                    clr = colors_binary[self.obstacles[y, x]]
                    mix.putpixel((x, y), clr)

        #path
        for particle in self.particles:
            clr = colors[particle.color]

            if self.start_point_viz:    
                mix.putpixel((int(particle.x), int(particle.y)), clr) #for starting points
            else:
                for p in particle.path: #for path
                    x = int(p[0])
                    y = int(p[1])

                    if 0 <= x < self.width and 0 <= y < self.height:
                        mix.putpixel((x, y), clr)

        if self.makeGif: self.gif.append(mix)
        if self.savePhoto: mix.save(self.data_path + 'mcl.jpeg')
    
    def createGif(self, frames):
        frame_one = frames[0]
        frame_one.save(self.data_path + "mcl.gif", format="GIF", append_images=frames,
               save_all=True, duration=100, loop=0)


if __name__ == '__main__':
    map = MapInterpolation()
    mc = MonteCarlo(map.generateVectors(map.mcl_coords), map.binary_array, map.map_width, map.map_height, map.data_path)
    start_time = time.time()
    mc.medial_axis_weight()
    # mc.set_particles()
    # mc.run_mcl()
    print("--- %s min ---" % ((time.time() - start_time) / 60))



    
