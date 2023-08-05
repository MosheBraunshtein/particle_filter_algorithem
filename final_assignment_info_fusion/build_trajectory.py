import random
import matplotlib.pyplot as plt
import numpy as np

class BuildTrajectory:
    def __init__(self,x_max,x_min,y_max,y_min):
        self.x_max = x_max
        self.x_min = x_min
        self.y_max = y_max
        self.y_min = y_min
        self.x_trajectory = []
        self.y_trajectory = []
        self.points = []
        self.k_list = []
    
    def generate_beacons(self):
        # Plot the points
        self.points = [(random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max)) for _ in range(10)]
        # plt.scatter(*zip(*self.points), color='red')
        # plt.scatter(self.points[0][0],self.points[0][1], color='black')
        # plt.scatter(self.points[9][0],self.points[9][1], color='blue')
    def plot_beacons(self):
        plt.scatter(*zip(*self.points), color='red')
        plt.scatter(self.points[0][0],self.points[0][1], color='black')
        plt.scatter(self.points[9][0],self.points[9][1], color='blue')
    def getSteps(self,L):
        steps = 10
        while L/steps > np.sqrt(2):
            steps += 2
        return steps
    def generate_trajectory(self):
        k=0
        for i in range(len(self.points) - 1):
            self.k_list.append(k)
            x1, y1 = self.points[i]
            x2, y2 = self.points[i + 1]

            L = (np.sqrt(((x2-x1)**2)+((y2-y1)**2)))**2
            steps = self.getSteps(L)
            k += steps
            dx = (x2 - x1) / steps  # Step size for x-coordinate
            dy = (y2 - y1) / steps # Step size for y-coordinate
            
            x_traj = [x1 + j * dx for j in range(steps+1)]
            y_traj = [y1 + j * dy for j in range(steps+1)]

            self.x_trajectory = self.x_trajectory + x_traj
            self.y_trajectory = self.y_trajectory + y_traj
        self.k_list.append(len(self.x_trajectory))

    def get_points(self):
        return self.x_trajectory,self.y_trajectory
    def get_beacons_location(self):
        return self.points
    def get_beacons_k(self):
        return self.k_list
    
    def plot_trajectory_and_beacons(self):
        plt.figure('trajectory')
        plt.scatter(*zip(*self.points), color='red')
        plt.scatter(self.points[0][0],self.points[0][1], color='black')
        plt.scatter(self.points[9][0],self.points[9][1], color='blue')
        plt.plot(self.x_trajectory,self.y_trajectory)
        plt.title('trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()