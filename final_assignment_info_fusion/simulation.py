import random
import numpy as np
import matplotlib.pyplot as plt
import math
from build_trajectory import BuildTrajectory
from initial_particles import InitialParticles

#number of particles
N = 1000
x_max , y_max = 30,40
x_min , y_min = 0,0

trajectory = BuildTrajectory(x_max,x_min,y_max,y_min)
trajectory.generate_beacons()
trajectory.generate_trajectory()
x_trajectory,y_trajectory = trajectory.get_points()
k_range = len(x_trajectory)
# trajectory.plot_trajectory_and_beacons()
#######################################
def d(beacon_i_x,beacon_i_y,xk,yk):
    return (np.sqrt(((beacon_i_x-xk)**2)+((beacon_i_y-yk)**2)))**2
def maximum_likelihood(d_real,d_excepted):
    return math.exp(-0.5*((d_real-d_excepted)**2)*0.4)
def sum_weights(particles_after_update_weights):
    sum = 0
    for p in particles_after_update_weights:
        px_k, py_k, weight_k = p
        sum = sum + weight_k
    return sum
def reWeight(particles_after_update_weights,weights_sum):
    particles_after_norm_weights = []
    for p in particles_after_update_weights:
        px_k, py_k, weight_k = p
        norm_weight = weight_k/weights_sum
        particles_after_norm_weights.append((px_k,py_k,norm_weight))
    return particles_after_norm_weights
def compute_estimation(particles):
    xk_estimated = 0
    yk_estimated = 0
    for p in particles:
        px_k, py_k, weight_k = p
        xk_estimated = xk_estimated + px_k*weight_k
        yk_estimated = yk_estimated + py_k*weight_k 
    return xk_estimated, yk_estimated
def find_beacons_in_range(x_robot_location,y_robot_location,beacons):
    beacons_filtered = []
    for beacon in beacons:
        x_beacon,y_beacon = beacon
        distance = d(x_beacon,y_beacon,x_robot_location,y_robot_location)
        if(distance<=2):
            beacons_filtered.append(beacon)
    return beacons_filtered
def propagate_particles(x_traj_k,x_traj_kplus1,y_traj_k,y_traj_kplus1,particles):
    particles_after_propagation = []
    for p in particles:
        wk = np.random.normal(np.zeros(2),np.sqrt(0.1))
        px_k, py_k, weight_k = p
        px_Kplus1 = px_k + x_traj_kplus1-x_traj_k + wk[0] 
        py_Kplus1 = py_k + y_traj_kplus1-y_traj_k + wk[1] 
        particles_after_propagation.append((px_Kplus1,py_Kplus1,weight_k))
    return particles_after_propagation
def update_weights(beacons,particles,x_traj_k,y_traj_k):
    particles_after_update_weights = []
    for p in particles:
        px_k, py_k, weight_k = p
        w=1
        for beacon in beacons:
            beacon_i_x, beacon_i_y = beacon
            di_real = d(beacon_i_x,beacon_i_y,x_traj_k,y_traj_k)
            di_excepted = d(beacon_i_x,beacon_i_y,px_k,py_k)
            # w += maximum_likelihood(di_real,di_excepted)
            mle = maximum_likelihood(di_real,di_excepted)
            w = w*mle
        compute_weight = w*weight_k
        particles_after_update_weights.append((px_k, py_k, compute_weight))
    return particles_after_update_weights
def check_for_resampling(particles):
    resampling = False
    sum = 0
    for p in particles:
        px ,py ,weight = p
        sum += weight**2 
    n_eff = 1/sum
    if((1/10)*len(particles)>=n_eff):
        resampling = True  
    return resampling
def resampling(particles):
    particles_after_resampling = []
    N = len(particles)
    for p in particles :
        px, py, pw = p
        m = math.ceil(pw*N)
        for _ in range(m-1):
                new_p = (px,py,1/N)
                particles_after_resampling.append(new_p)
    return particles_after_resampling
########################################


######################### PF ####################
beacons = trajectory.get_beacons_location()
particles_after_propagation = []
relevant_beacons = []
particles_after_update_weights = []
xk_estimates = []
yk_estimates = []
mse = []
# #build the points
initialParticles = InitialParticles(N,beacons,x_min,x_max,y_min,y_max)
particles = initialParticles.regularMethod()
frames = []
plt.figure()
plt.xlim(0, 30)
plt.ylim(0, 40)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Propagation of 100 Dots')
initial_xlim = plt.xlim()
initial_ylim = plt.ylim()
for k in range(k_range-1):
    plt.clf()
    plt.scatter(*zip(*beacons), color='red')
    plt.scatter(beacons[0][0],beacons[0][1], color='black')
    plt.scatter(beacons[9][0],beacons[9][1], color='blue')
    plt.plot(x_trajectory,y_trajectory)
    pxs = []
    pys = []
    for p in particles:
        px,py,w = p
        pxs.append(px)
        pys.append(py)
    plt.plot(pxs, pys, 'o', markersize=1)
    plt.plot(x_trajectory[k],y_trajectory[k], 'o', markersize=5)
    plt.xlim(initial_xlim)
    plt.ylim(initial_ylim)
    plt.pause(0.1)
    frames.append(plt.gcf())
    resample = False
    particles_after_propagation = propagate_particles(x_trajectory[k],x_trajectory[k+1],y_trajectory[k],y_trajectory[k+1],particles)
    relevant_beacons = find_beacons_in_range(x_trajectory[k+1],y_trajectory[k+1],beacons)
    if(len(relevant_beacons)>0):
        particles_after_update_weights = update_weights(relevant_beacons,particles_after_propagation,x_trajectory[k],y_trajectory[k])
        weights_sum = sum_weights(particles_after_update_weights)
        particles_after_norm_weights = reWeight(particles_after_update_weights,weights_sum)
        xk_estimated, yk_estimated = compute_estimation(particles_after_norm_weights)
        resample = check_for_resampling(particles_after_norm_weights)
        if(resample):
            particles_after_resampling = resampling(particles_after_norm_weights)
            particles = particles_after_resampling
        else:
            particles = particles_after_norm_weights
    else:
        xk_estimated, yk_estimated = compute_estimation(particles_after_propagation)
        particles = particles_after_propagation
    plt.plot(x_trajectory,y_trajectory,color='green')
    xk_estimates.append(xk_estimated)
    yk_estimates.append(yk_estimated)
    mse.append(d(xk_estimated,yk_estimated,x_trajectory[k+1],y_trajectory[k+1])) 

from matplotlib.animation import PillowWriter

writer = PillowWriter(fps=10)  # Adjust the frames per second (fps) as needed
animation_filename = 'C:/Users/win10/Desktop/BGU_MSE/simester_1/information fusion/project/dots_propagation_animation.mp4'
writer.setup(fig=plt.figure(), outfile=animation_filename, dpi=100)
for frame in frames:
    writer.grab_frame()
plt.show()





































