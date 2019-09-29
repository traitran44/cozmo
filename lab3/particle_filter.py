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
    measured_particles = resample(particles, weights, len(particles))

    return particles


# Resample new particles based on weights
def resample(particles, weights, new_size):  # Trai
    """
    :param particles: [ (x, y, heading), ...., (..) ]
    :param weights: [0.2, 0.4, 0.7, 1.0]
    :return:  new particles [ (x, y, heading), ...., (..) ]
    """
    new_particles = []
    # Prefix sum prob, setup probability range for each particle
    for i in range(1, len(weights)):
        weights[i] = weights[i - 1] + weights[i]

    for i in range(new_size):
        rand = random.random()
        for i in range(len(weights)):
            if rand <= weights[i]:
                chosen = particles[i]
                new_particles.append(chosen)
                break

    if len(new_particles) < new_size:
        print("New particles size cannot be less than particles size!")

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
        distances = [ [ 0 for a in range(len(particle_markers)) ] for b in range(len(robot_markers)) ]
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
def normalize_weights(weights):  # Trai
    total = sum(weights)
    norm_w = []
    for w in weights:
        norm_w.append(
            w / total
        )
    return norm_w


# Use equation in the slide
def weight_update(pairs):
    """
    :param pairs:
    :return:
    """
    print("Pairs", pairs)
    w = 1
    return w


if __name__ == "__main__":
    # test resample
    new_p = resample(
        [0, 1, 2, 4],
        [0.1, 0.2, 0.4, 0.3],
        1000000
    )
    freq = {}
    for p in new_p:
        if p not in freq:
            freq[p] = 0
        freq[p] += 1

    for k in freq.keys():
        freq[k] = freq[k] / 1000000
        print("Freq ", k, " is ", freq[k])

    # test normalize_weights
    norm_w = normalize_weights(
        [2, 3, 10, 22, 32, 1]
    )
    print("Sum Norm weight: ", sum(norm_w))

    # test get_match_markers
    test1 = [(2, 5, 3), (8, 4, 4)]
    test2 = [(4, 6, 6), (2, 0, 1), (4, 3, 7)]
    pairs = get_match_markers(test1, test2)
    print("pairs: ", pairs)
