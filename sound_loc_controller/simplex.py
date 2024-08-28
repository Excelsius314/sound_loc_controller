import math
import numpy as np


def sample_in_donut(min_rad, max_rad, n_samples):

    sampled_r = np.random.rand(n_samples) * max_rad + min_rad
    sampled_angle = (np.random.rand(n_samples) -0.5) * 2*math.pi
    return sampled_r.reshape(-1, 1) * np.concatenate((np.cos(sampled_angle).reshape(-1, 1), 
                                         np.sin(sampled_angle).reshape(-1, 1)), axis=1)


def determine_direction(simplex, formation_center, robot_distance, n_samples=10000):

    A, b = simplex
    # sample points outside of circle (M= formation_center, radius = robot_distance)
    outer_points = sample_in_donut(robot_distance, 2*robot_distance, n_samples) + formation_center.reshape(-1, 1).transpose()
    valid_outer_points = outer_points[((A @ outer_points.transpose()) <= b).transpose().all(axis=1)]

    return np.average(valid_outer_points, axis=0) - formation_center, outer_points


def ILD(db_2, db_1):
  return 1 if db_1 >= db_2 else -1


# microphones: [{"pos": np.array([x,y]), "dB": db} ...]
def determine_sound_source_halfspaces(all_microphones, use_robot_average=False):

    separations = []

    if use_robot_average:

        microphones_avg = []        
        for i in range(int(len(all_microphones)/3)):
            robot_micros = all_microphones[i*3: (i+1)*3]

            pos_avg = np.average(np.array([robot_micro["pos"] for robot_micro in robot_micros]).reshape(-1, 2), axis=0)
            db_avg = np.average(np.array([robot_micro["dB"] for robot_micro in robot_micros]))
            microphones_avg.append({"pos": pos_avg,"dB": db_avg})
        
        all_microphones = microphones_avg

    for i in range(int(len(all_microphones)/3)):

        microphone_group = all_microphones[i*3: (i+1)*3]
        pairs = [(1, 0), (0, 2), (1, 2)]
        for pair in pairs:
            
            main_microphone = microphone_group[pair[0]]
            compared_microphone = microphone_group[pair[1]]
            
            direction = ILD(main_microphone["dB"], compared_microphone["dB"])

            #print("Pair {} with decision {}: main {} with dB {}; compared {} with dB {}".format(pair, direction,
            #                                                                                    pair[0], main_microphone["dB"],
            #                                                                                    pair[1], compared_microphone["dB"]))
            ray, support_vector = get_decision_boundary(main_microphone["pos"],
                                                        compared_microphone["pos"],
                                                        direction)
            std_separation = standard_line_form_from_parameterized(ray, support_vector)
            separations.append({"std_form": std_separation, "parameterized": (ray, support_vector, direction)})

    return separations

    
def standard_line_form_from_parameterized(ray, support_vector):

    # [[x], [y]] = t * ray + support_vector, t e R
    # Ax + By = C
    A =  -ray[1] # -slope
    B = ray[0]  # 1
    C = ray[0] * support_vector[1] - ray[1] * support_vector[0]  # support_vector[1] - slope * support_vector[0]

    return [A, B, C]


def get_decision_boundary(p1, p2, ild):

    rot_mat = np.array([[math.cos(math.pi/2), math.sin(ild *math.pi/2)],
                        [-math.sin(ild * math.pi/2), math.cos(math.pi/2)]])

    return rot_mat @ (p1 - p2), (p1 - p2) / 2 + p2



def aggregate_separations_to_simplex(separations):

    A = np.zeros((len(separations), 2))
    b = np.zeros((len(separations), 1))

    for i, separation in enumerate(separations):

        A_sep, B_sep, C_sep = separation["std_form"]
        A[i, 0] = A_sep
        A[i, 1] = B_sep
        b[i] = C_sep

    return A, b