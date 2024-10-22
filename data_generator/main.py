import numpy as np
import random
import json
import math
import copy


# Function to save the instance to a JSON file
def save_to_json(points, obstacles, filename):
    data = {
        "obstacles": obstacles,
        "points": points
    }

    with open(filename, 'w') as file:
        json.dump(data, file, indent=2)


# Function to check for collisions
def line_segment_sphere_intersection(P1, P2, C, r):
    if P1 == P2:
        return False
    P1 = np.array(P1)
    P2 = np.array(P2)
    C = np.array(C)
    direction = P2 - P1
    offset = P1 - C
    a = np.dot(direction, direction)
    b = 2 * np.dot(offset, direction)
    c = np.dot(offset, offset) - r**2
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return False
    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)
    if 0 <= t1 <= 1 or 0 <= t2 <= 1:
        return True
    return False


# Function to calculate the distance between two points
def calculate_distance(P1, P2):
    return ((P1[0] - P2[0])**2 + (P1[1] - P2[1])**2 + (P1[2] - P2[2])**2)**0.5


# Function to generate a problem instance
def generate_instance(number_of_points, number_of_obstacles, min_coord, max_coord):
    points = []
    obstacles = []

    indexes_of_points = [x for x in range(number_of_points)]
    permutation = copy.deepcopy(indexes_of_points)
    random.shuffle(permutation)
    permutation.append(permutation[0])

    max_edges = math.factorial(number_of_points) / (2 * math.factorial(number_of_points - 2))
    max_obstacles = max_edges - number_of_points

    if number_of_obstacles > max_obstacles:
        number_of_obstacles = max_obstacles

    for _ in range(number_of_points):
        point = [random.randint(min_coord, max_coord), random.randint(min_coord, max_coord), random.randint(min_coord, max_coord)]
        points.append(point)

    pairs = [(permutation[i], permutation[i + 1]) for i in range(len(permutation) - 1)]
    print(pairs)
    used_pairs_of_indexes = []

    while len(obstacles) < number_of_obstacles:
        obstacle = None
        while not obstacle:
            index_of_P1, index_of_P2 = random.sample(indexes_of_points, 2)
            if ((index_of_P1, index_of_P2) not in pairs or (index_of_P2, index_of_P1) not in pairs) and (index_of_P1, index_of_P2) not in used_pairs_of_indexes and (index_of_P2, index_of_P1) not in used_pairs_of_indexes:
                P1 = points[index_of_P1]
                P2 = points[index_of_P2]

                t = random.uniform(0, 1)
                x = P1[0] + t * (P2[0] - P1[0])
                y = P1[1] + t * (P2[1] - P1[1])
                z = P1[2] + t * (P2[2] - P1[2])
                M = [x, y, z]

                max_r = float("inf")
                for point in points:
                    distance = calculate_distance(M, point)
                    if distance < max_r:
                        max_r = distance
                if max_r > ((max_coord - min_coord) / 40) + 0.1:
                    r = random.uniform((max_coord - min_coord) / 40, max_r - 0.1)
                    if not any(line_segment_sphere_intersection(points[pair[0]], points[pair[1]], M, r) for pair in pairs) and not any((calculate_distance(obstacle[0], M) < max(obstacle[1], r)) for obstacle in obstacles):
                        obstacle = [M, r]
        used_pairs_of_indexes.append((index_of_P1, index_of_P2))
        obstacles.append(obstacle)
    return points, obstacles


if __name__ == '__main__':
    number_of_points = 10
    number_of_obstacles = 9
    min_coord = -100
    max_coord = 100
    instance_file_path = '../instance.json'

    points, obstacles = generate_instance(number_of_points, number_of_obstacles, min_coord, max_coord)
    print(f"points={points}")
    print(f"obstacles={obstacles}")
    save_to_json(points, obstacles, instance_file_path)
