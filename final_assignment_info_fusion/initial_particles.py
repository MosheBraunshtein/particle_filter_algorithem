import random
import matplotlib.pyplot as plt
import numpy as np

class InitialParticles:
    def __init__(self,n,beacons,x_min,x_max,y_min,y_max):
        self.particles = []
        self.N = n
        self.beacons = beacons
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max


    def regularMethod(self):
        self.particles = [(random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max), 1/self.N) for _ in range(self.N)]
        return self.particles

    def betterMethod(self):
        x_values = [point[0] for point in self.beacons]
        y_values = [point[1] for point in self.beacons]
        # Find maximum and minimum values
        x_max = max(x_values)
        x_min = min(x_values)
        y_max = max(y_values)
        y_min = min(y_values)
        self.particles = [(random.uniform(x_min-2, x_max+2), random.uniform(y_min-2, y_max+2), 1/self.N) for _ in range(self.N)]
        return self.particles