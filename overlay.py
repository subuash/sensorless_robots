#!/usr/bin/env python

import rosbag
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from PIL import Image
import pandas as pd

class Particle():
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        self.color = np.random.randint(0, 10)
        self.start = (x, y)
        self.path = []

class MapInterpolation():

    def __init__(self):

        self.bag_name = 'path2'

        self.map_width = 384
        self.map_height = 384 
        self.resolution = 0.05000000074505806
        self.origin_x = -10.0 
        self.origin_y = -10.0  
        self.trials = 1
        self.variations = 1

        #turtlebot3 stage only
        self.bounds_x = [142, 252]
        self.bounds_y = [150, 250]

        self.only_start_viz = True

        self.paths = []
        self.path_coords = []
        self.map_data = []
        self.map_coords = []
        self.diff_vectors = []
        self.precise_diff_vectors = []

        self.map_array = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        self.bag = rosbag.Bag(self.bag_name + '.bag')
        self.viable = set()

        self.readMapData()
        self.binary_array = np.where(self.map_data == 100, 1, 0)

        self.readBag()
        self.generateVectors()
        self.generatePreciseVectors()

        self.original_start = self.map_data[0]
        start_points = self.random_start(self.trials)

        for start_point in start_points:
            self.randomPath(start_point, self.variations)

        self.generateVisuals()

        # print("\nViable start positions: " + str(len(self.viable)) + ", " + str((len(self.viable)/(self.trials)) * 100) + "%")

    def random_start(self, trials):
        res = []
        for _ in range(trials):
            # r_x = int(np.random.uniform(0, self.map_width))
            # r_y = int(np.random.uniform(0, self.map_height)) 
            r_x = int(np.random.uniform(self.bounds_x[0], self.bounds_x[1])) ##FOR TURTLEBOT3 STAGE ONLY
            r_y = int(np.random.uniform(self.bounds_y[0], self.bounds_y[1]))
            res.append((r_x, r_y))

        return res

    def collision(self, path):

        array = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        for x, y in path:
            # if 0 <= x < self.map_width and 0 <= y < self.map_height:
            if self.bounds_x[0] <= x < self.bounds_x[1] and self.bounds_y[0] <= y < self.bounds_y[1]: ##FOR TURTLEBOT3 STAGE ONLY
                array[y, x] = 1
            else:
                return False
            
        logical_and = np.logical_and(array, self.binary_array)
        points = np.argwhere(logical_and == True)

        if points.size == 0:
            return True

        return False

    def readBag(self):
        for topic, msg, t in self.bag.read_messages(topics='/path'):
            x = msg.__getattribute__('x')
            y = msg.__getattribute__('y')
            self.path_coords.append((x, y))

    def generateVectors(self):

        #original points
        for x, y in self.path_coords:
            map_x = int((x - self.origin_x) / self.resolution)  
            map_y = int((y - self.origin_y) / self.resolution)
            self.map_coords.append((map_x, map_y))

        #vectors
        for x in range(len(self.map_coords) - 1):
            vector = np.array([self.map_coords[x+1][0] - self.map_coords[x][0], self.map_coords[x+1][1] - self.map_coords[x][1]])
            mag = np.linalg.norm(vector)
            ang = np.arctan2(vector[1], vector[0])
            self.diff_vectors.append((mag, ang))
        
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
        self.map_data = np.array(data.data, dtype=np.int8).reshape(384, 384)
        
        # df = pd.DataFrame(self.map_data)
        # df.to_csv('map_data.csv', index=False)

    def readMapData(self):
        # rospy.init_node('map_subscriber')

        # rospy.Subscriber('/map', OccupancyGrid, self.readMapDataCallback)
        # rospy.spin()
        
        self.map_data = pd.read_csv('map_data.csv').to_numpy()
    
    def generateVisuals(self):

        colors = {
            0: (255, 255, 255),       # White
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

    def __init__(self, num_particles, diff_vectors, obstacles, width, height):
        self.particles = []
        self.num_particles = num_particles
        self.diff_vectors = diff_vectors
        self.obstacles = obstacles
        self.width = width
        self.height = height

        self.bounds_x = [142, 252]
        self.bounds_y = [150, 250]

    def set_particles(self):
        for _ in range(self.num_particles):
            x = int(np.random.uniform(self.bounds_x[0], self.bounds_x[1])) #change to proper bounds when not using turtlebot3
            y = int(np.random.uniform(self.bounds_y[0], self.bounds_y[1])) #and remove the parametres, they aren't neccesary
            theta = np.random.uniform(0, 2 * np.pi)
            weight = 1.0 / self.num_particles
            self.particles.append(Particle(x, y, theta, weight))

    def run_mcl(self):
        for mag, ang in self.diff_vectors:
            self.motion_update(mag, ang)
            self.collision_update()
        self.print_map()
            
    def motion_update(self, mag, ang):
        for i in range(len(self.particles)):
            p = self.particles[i]
            # print([(x.x, x.y) for x in p.path])
            # print(p.x, p.y)
            p.path.append(p)
            r_mag = np.random.uniform(0.75, 1.25)
            r_ang = np.random.uniform(-np.pi / 6, np.pi / 6)

            adjusted_mag = mag * r_mag
            adjusted_ang = ang + r_ang + p.theta
            # print(adjusted_mag, adjusted_ang)
            # print(adjusted_mag * np.cos(adjusted_ang), int(adjusted_mag * np.cos(adjusted_ang)))
            # print((p.x,p.y), int(adjusted_mag * np.cos(adjusted_ang)))
            # oldx = p.x
            # oldy = p.y
            # print((self.particles[i].x, self.particles[i].y))
            p.x = p.x + int(adjusted_mag * np.cos(adjusted_ang))
            p.y = p.y + int(adjusted_mag * np.sin(adjusted_ang))
            # print((self.particles[i].x, self.particles[i].y))

            # print((oldx - self.particles[i].x, oldy - self.particles[i].y), (p.x - oldx, p.y-oldy))
            self.particles[i] = p
        
    def collision_update(self):
        t_particles = []
        for particle in self.particles:
            if self.bounds_x[0] <= particle.x <= self.bounds_x[1] \
            and self.bounds_y[0] <= particle.y <= self.bounds_y[1] \
            and self.obstacles[particle.y, particle.x] == 0:
                t_particles.append(particle)

        self.particles = t_particles
        self.num_particles = len(self.particles)
    
    def print_map(self):
        mim = Image.new('RGB', (self.width, self.height), color='white')

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
                    mim.putpixel((x, y), clr)

        #path
        for particle in self.particles:
            clr = colors[particle.color]
            # print('--------------------------------------------------------')
            print(len(particle.path))
            for p in particle.path:
                x = p.x
                y = p.y
                # print(x, y)
                mim.putpixel((x, y), clr)
        
        mim.save('mcl.jpeg')

if __name__ == '__main__':
    map = MapInterpolation()
    map.precise_diff_vectors = np.add.reduceat(map.precise_diff_vectors, np.arange(0, len(map.precise_diff_vectors), 5))
    # for i in range(len(map.precise_diff_vectors)):
    #     print(map.precise_diff_vectors[i])
    # print(len(map.precise_diff_vectors))
    mc = MonteCarlo(10, map.precise_diff_vectors, map.binary_array, map.map_width, map.map_height)
    mc.set_particles()
    mc.run_mcl()

    