from grid import *
from particle import Particle
from utils import *
import setting
import numpy as np
import random
np.random.seed(setting.RANDOM_SEED)
random.seed(setting.RANDOM_SEED)


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    add_odometry_noise(odom, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
    motion_particles = particles
    for i in range(len(motion_particles)):
        dx, dy = rotate_point(odom[0], odom[1], motion_particles[i].h)
        motion_particles[i].x += dx
        motion_particles[i].y += dy
        motion_particles[i].h = motion_particles[i].h + odom[2]
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    weights = [0] * len(particles)
    for i in range(len(particles)):
        particle_markers = particles[i].read_markers(grid)
        pairs = get_match_markers(measured_marker_list, particle_markers)
        weights[i] = weight_update(pairs)

    weights = normalize_weights(weights)
    measured_particles = resample(particles, weights)

    return measured_particles


# Resample new particles based on weights
def resample(particles, weights):
    new_particles = []
    return new_particles

# Pair each marker to its corresponding match (with noise)
def get_match_markers(robot_markers, particle_markers):
    if len(robot_markers) > len(particle_markers):
        min_markers = len(particle_markers)
    else:
        min_markers = len(robot_markers)
    if min_markers == 0:
        return []
    pairs = [0] * min_markers
    for i in range(min_markers):
        distances = [[0] * len(particle_markers)] * len(robot_markers)
        for j in range(len(robot_markers)):
            for k in range(len(particle_markers)):
                distances[j][k] = grid_distance(robot_markers[j][0], robot_markers[j][1], particle_markers[k][0], particle_markers[k][1])
        flattened_distances = [x for y in distances for x in y]
        ind = flattened_distances.index(min(flattened_distances))
        pairs[i] = robot_markers[int(ind/len(particle_markers))], particle_markers[ind%len(particle_markers)]
        robot_markers.pop(int(ind/len(particle_markers)))
        particle_markers.pop(ind%len(particle_markers))
    return pairs


# Normalize s.t sum to 1
def normalize_weights(weights):
    norm_w = []
    return norm_w


# Use equation in the slide
def weight_update(pairs):
    w = 1
    return w
