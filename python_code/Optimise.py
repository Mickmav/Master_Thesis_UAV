import random
import math
import subprocess
import json
import dubins
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from copy import deepcopy
from itertools import permutations

from Utils import boustrophedon_cells


def cpp_opti(path, matrix, type_of_optimisation, cpp_executable_path, points=None, turning_radius=0):
    """
    Apply a given cpp program to perform the optimisations functions more optimally
    :param path: a given starting solution
    :param matrix: a matrix of distance between the points
    :param type_of_optimisation: the type of optimisation need (0 = SA / 1 = GA / 2 = SLS)
    :param cpp_executable_path: path of the cpp compiled program
    :param points:
    :param turning_radius:
    :return: The final solution
    """
    if points is None:
        points = [[0, 0]]
    else:
        # Convert points to list if it's a numpy array
        if isinstance(points, np.ndarray):
            points = points.tolist()
    if type_of_optimisation < 3:
        turning_radius = 0
    # Prepare input data as a dictionary
    if matrix is None:
        matrix = [[]]
    else:
        matrix = matrix.tolist()
    input_data = {"list": path, "matrix": matrix, "type": type_of_optimisation, "points": points,
                  "turning_radius": turning_radius}

    # Serialize the data to JSON
    input_json = json.dumps(input_data)

    try:
        # Use subprocess to run the C++ executable and communicate using pipes
        completed_process = subprocess.run(
            [cpp_executable_path],
            input=input_json.encode(),  # Pass input as bytes
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True
        )

        # Parse the output JSON from the C++ program
        output_data = json.loads(completed_process.stdout)
        output_list = output_data.get("output_list", [])
        return output_list

    except subprocess.CalledProcessError as e:
        print("Error executing C++ code:", e)


def calculate_cost(path, matrix):
    """
    Cost function of the different algorithms
    :param path: a given solution
    :param matrix: matrix of distance between the points
    :return: cost of the solution
    """
    cost = 0
    for index in range(len(path)):
        cost += matrix[path[index]][path[(index + 1) % len(path)]]
    return cost


# Function to compute headings between points
def compute_headings(points, order):
    headings = []
    size = len(order)
    for i in range(size):
        p1 = points[order[i]]
        p2 = points[order[(i + 1) % size]]

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        heading = np.arctan2(dy, dx)
        headings.append(heading)

    return headings


# Function to compute headings between points
def compute_headings_2(points, order):
    headings = []
    size = len(order)
    for i in range(size):
        p0 = points[order[i - 1]] if i > 0 else points[order[-1]]
        p1 = points[order[i]]
        p2 = points[order[i + 1]] if i < size - 1 else points[order[0]]

        # Vectors from p0 to p1 and p1 to p2
        v1 = np.array([p1[0] - p0[0], p1[1] - p0[1]], dtype=np.float64)
        v2 = np.array([p2[0] - p1[0], p2[1] - p1[1]], dtype=np.float64)

        # Normalize vectors
        v1 /= np.linalg.norm(v1)
        v2 /= np.linalg.norm(v2)

        # Calculate the sum of vectors
        v_sum = v1 + v2

        if np.linalg.norm(v_sum) == 0:
            # Handle the case where v1 and v2 are exact opposites
            # print(f"Vectors v1 and v2 are opposites at index {i}: v1={v1}, v2={v2}")
            heading = np.arctan2(v1[1], v1[0])  # Arbitrary choice: use the direction of v1
        else:
            # Calculate the bisector vector
            bisector = v_sum / np.linalg.norm(v_sum)

            # Calculate the heading angle
            heading = np.arctan2(bisector[1], bisector[0])

        headings.append(heading)

    return headings


def compute_total_dubins_path_length(points, order, turning_radius):
    """
    Optimized function to compute only the total length of the Dubins paths
    :param points: the x,y positions of all the points
    :param order: the order in which the points are crossed
    :param turning_radius: the tuning radius
    :return: length of the path
    """
    # Compute headings
    headings = compute_headings_2(points, order)

    total_length = 0.0

    for i in range(len(order)):
        idx_start = order[i]
        idx_end = order[(i + 1) % len(order)]

        # Start and end configurations with computed headings
        start_point = (points[idx_start][0], points[idx_start][1], headings[i])
        end_heading = headings[i + 1] if i + 1 < len(headings) else 0  # Use the next heading if available
        end_point = (points[idx_end][0], points[idx_end][1], end_heading)

        # Dubins path computation
        path = dubins.shortest_path(start_point, end_point, turning_radius)
        path_length = path.path_length()  # Get the length of the Dubins path

        total_length += path_length

    return total_length


# Function to compute Dubins paths and their total length
def compute_dubins_paths_and_length(points, order, turning_radius):
    """
    Optimized function to compute only the total length of the Dubins paths
    :param points: the x,y positions of all the points
    :param order: the order in which the points are crossed
    :param turning_radius: the tuning radius
    :return: length of the path
    """

    # Compute headings
    headings = compute_headings_2(points, order)

    paths = []
    total_length = 0.0

    for i in range(len(order)):
        idx_start = order[i]
        idx_end = order[(i + 1) % len(order)]

        # Start and end configurations with computed headings
        start_point = (points[idx_start][0], points[idx_start][1], headings[i])
        end_heading = headings[i + 1] if i + 1 < len(headings) else headings[0]  # Use the next heading if available
        end_point = (points[idx_end][0], points[idx_end][1], end_heading)

        # Dubins path computation
        path = dubins.shortest_path(start_point, end_point, turning_radius)
        configurations, _ = path.sample_many(0.1)  # Sample points along the path
        path_length = path.path_length()  # Get the length of the Dubins path

        paths += configurations
        total_length += path_length

    return total_length, paths


def optimise_dubins(initial_solution, points, turning_radius, type_of_optimisation=0):
    new_sol = []
    if type_of_optimisation == 0:
        new_sol = simulated_annealing(initial_solution,
                                      lambda sol: compute_total_dubins_path_length(points, sol, turning_radius),
                                      random_exchange)
    if type_of_optimisation == 1:
        new_sol = genetic_algorithm(len(initial_solution),
                                    lambda sol: compute_total_dubins_path_length(points, sol, turning_radius),
                                    100)
    if type_of_optimisation == 2:
        new_sol = simple_local_search(initial_solution,
                                      lambda sol: compute_total_dubins_path_length(points, sol, turning_radius))

    return new_sol


def optimise(initial_solution, matrix, type_of_optimisation=0):
    """
     Apply a given python algorithm to perform the optimisations functions
    :param initial_solution: a given starting solution
    :param matrix: a matrix of distance between the points
    :param type_of_optimisation: the type of optimisation need (0 = SA / 1 = GA / 2 = SLS)
    :return: The final solution
    """
    new_sol = []
    if type_of_optimisation == 0:
        new_sol = simulated_annealing(initial_solution, lambda sol: calculate_cost(sol, matrix), random_exchange)
    if type_of_optimisation == 1:
        new_sol = genetic_algorithm(len(initial_solution), lambda sol: calculate_cost(sol, matrix), 100)
    if type_of_optimisation == 2:
        new_sol = simple_local_search(initial_solution, lambda sol: calculate_cost(sol, matrix))

    print(calculate_cost(new_sol, matrix), new_sol)
    return new_sol


def compute_cooling_rate(size):
    """
    Compute a cooling rate based of the size of a solution
    :param size: size of a solution
    :return: cooling rate
    """
    cooling_rate_str = "0." + "9" * int(math.log(size))
    return float(cooling_rate_str)


def random_exchange(solution):
    """
    Perform a random 2-exchange in a given solution
    :param solution:
    :return: a modified solution
    """
    start = random.randint(0, len(solution) - 1)
    possible_choice = solution.copy()
    possible_choice.pop(start)
    end = random.choice(possible_choice)
    if start > end:
        start, end = end, start
    part1 = solution[:start]
    part2 = solution[start:end + 1]
    part3 = solution[end + 1:]

    new_sol = part1 + part2[::-1] + part3
    return new_sol


def acceptance_probability(cost_diff, temperature):
    """
    Probability to accept worsening solution
    :param cost_diff:
    :param temperature:
    :return:
    """
    return math.exp(-cost_diff / temperature)


def simulated_annealing(initial_solution, cost_function, perturbation_function, temperature=900, min_temperature=0.009):
    """
    Perform a simulated annealing algorithm that performs random modifications on a solution and has a probability
    (based on a variable called temperature) to accept worse solutions to extend the search scope.
    :param initial_solution: initial order of points to visit
    :param cost_function: function to calculate the cost of a given solution
    :param perturbation_function: function to generate a new solution by perturbing the current solution
    :param temperature: starting temperature
    :param min_temperature: when temperature is too small, stop the process
    :return: the best solution obtained
    """
    # Initialize the process
    cooling_rate = compute_cooling_rate(len(initial_solution))
    current_solution = initial_solution
    best_solution = initial_solution
    best_cost = cost_function(best_solution)

    i = 0
    while temperature > min_temperature:
        # Modify the solution
        new_solution = perturbation_function(current_solution)  # Small random perturbation

        current_cost = cost_function(current_solution)
        new_cost = cost_function(new_solution)

        # Verify the solution and accept if conditions are met
        if new_cost < current_cost or random.random() < acceptance_probability(new_cost - current_cost, temperature):
            current_solution = new_solution

        # Change the best solution when a better solution is found
        if current_cost < best_cost:
            best_solution = current_solution
            best_cost = current_cost

        # Update temperature
        temperature *= cooling_rate

        i += 1
        if i >= 20000:
            i = 0
            print(temperature, best_cost, best_solution)

    return best_solution


# Genetic Algorithm Components
def generate_random_solution(nb_points):
    """
    Generate a completely random solution
    :param nb_points: size of a solution
    :return: the random solution
    """
    return random.sample(range(nb_points), nb_points)


def find_best(initial_population, cost_function, size_returning_list):
    """
    Sort a list based on the cost function, then extract the "n best" solution
    :param initial_population: starting group of solution
    :param cost_function: function to calculate the cost of a given solution
    :param size_returning_list: number of solution to return
    :return: the n better solution in a sorted list
    """
    initial_population = sorted(initial_population, key=cost_function)
    return initial_population[:size_returning_list]


def cross_over(best, solutions):
    """
    Perform a mix between the bests solutions and all the solutions
    :param best: Group of the best solutions
    :param solutions: all the solutions
    :return: list of the new solutions
    """
    new_pop = []
    size_sol = len(solutions[0])
    # Take a solution as parent 2
    for parent2 in solutions:
        # Choose one of the best solution as parent 1
        parent1 = random.choice(best)
        while parent1 == parent2 and len(best) != 1:
            parent1 = random.choice(best)

        # Choose interval for the cross-over
        start = random.randint(0, size_sol - 2)
        end = random.randint(start + 1, size_sol - 1)

        child1 = [-1] * size_sol
        child2 = [-1] * size_sol

        child1[start:end + 1] = parent1[start:end + 1]
        child2[start:end + 1] = parent2[start:end + 1]
        # Cross the 2 solution
        for i in range(len(parent2)):
            if parent2[i] not in child1:
                j = 0
                while child1[j] != -1:
                    j += 1
                child1[j] = parent2[i]

            if parent1[i] not in child2:
                j = 0
                while child2[j] != -1:
                    j += 1
                child2[j] = parent1[i]

        new_pop.append(child1)
        new_pop.append(child2)

    return new_pop


def mutate(solutions, cost_function):
    """
    Perform a random permutation on all the solutions
    :param solutions: list of all the solution
    :param cost_function: function to calculate the cost of a given solution
    :return: a modified version of the received solution
    """
    new_solutions = []
    for solution in solutions:
        cost = cost_function(solution)
        new_solution = random_exchange(solution)
        new_cost = cost_function(new_solution)
        stop = 0
        while new_cost > cost and stop < 2:
            stop += 1
            new_solution = random_exchange(solution)
            new_cost = cost_function(new_solution)
        if new_cost < cost:
            solution = new_solution
        new_solutions.append(solution)
    return new_solutions


def genetic_algorithm(nb_points, cost_function, pop_size):
    """
    Perform a genetic algorithm with mutation, cross-over with survival type of selection
    :param nb_points: number of point in the solution
    :param cost_function: function to calculate the cost of a given solution
    :param pop_size: size of the population of solution
    :return: the best solution obtained
    """
    population = [generate_random_solution(nb_points) for _ in range(pop_size)]
    best_solution = find_best(population, cost_function, 1)[0]
    best_cost_ever = cost_function(best_solution)

    repetition = 0
    i = 0
    while repetition < nb_points + 10:
        best_pop = find_best(population, cost_function, int(pop_size / 10))
        cross_over_pop = cross_over(best_pop, population)
        mutated_pop = mutate(population, cost_function)
        mutated_cross = mutate(cross_over_pop, cost_function)
        population = population + cross_over_pop + mutated_pop + mutated_cross
        population = find_best([list(point) for point in set(tuple(solution) for solution in population)],
                               cost_function, pop_size)
        new_cost = cost_function(population[0])
        if new_cost == best_cost_ever:
            repetition += 1
        else:
            repetition = 0

        best_solution = population[0]
        best_cost_ever = new_cost

        i += 1
        if i >= 50:
            i = 0
            print(best_cost_ever)

    return best_solution


# Generic Local Search Methods
def simple_search(solution, cost_function):
    """
    Search using 2-exchange method with "best improvement" pivoting rule
    :param solution:
    :param cost_function: function to calculate the cost of a given solution
    :return: upgraded version of the input solution
    """
    nb_points = len(solution)
    best_cost = cost_function(solution)
    best_sol = solution
    for point in range(nb_points - 1):
        for second_point in range(point + 1, nb_points):
            part1 = solution[:point]
            part2 = solution[point:second_point + 1]
            part3 = solution[second_point + 1:]

            new_sol = part1 + part2[::-1] + part3
            new_cost = cost_function(new_sol)
            if new_cost < best_cost:
                best_sol = new_sol
                best_cost = new_cost
    return best_sol


def simple_local_search(solution, cost_function):
    """
    Perform a simple search on a solution until no upgrade is found
    :param solution:
    :param cost_function: function to calculate the cost of a given solution
    :return: the best solution obtained
    """
    solution_cost = cost_function(solution)
    new_cost = 0
    i = 0
    while solution_cost != new_cost:
        solution_cost = new_cost
        solution = simple_search(solution, cost_function)
        new_cost = cost_function(solution)

        i += 1
        if i == 20:
            i = 0
            print(solution_cost, solution)

    return solution


def adjust_line_segment(segment, turning_radius, direction):
    """
    Adjust the x-values of the segment endpoints based on the turning radius.
    :param segment:
    :param turning_radius:
    :param direction:
    :return:
    """
    adjusted_segment = []
    if len(segment) >= 2:
        p1, p2 = np.array(segment[0]), np.array(segment[-1])

        if direction == 1:  # Line moving right
            p1[0] += turning_radius
            p2[0] -= turning_radius
        else:  # Line moving left
            p1[0] -= turning_radius
            p2[0] += turning_radius

        # Ensure the adjusted points are within bounds
        adjusted_segment = [tuple(p1), tuple(p2)]
    return adjusted_segment


def boustrophedon_path(polygon, vision_radius, turning_radius):
    """
    Generate a boustrophedon path for covering the area within a polygon.
    :param polygon: The polygonal area to cover, defined by its boundary.
    :param vision_radius: Distance ahead the vehicle can "see" or consider for its next move.
    :param turning_radius: Minimum turning radius of the vehicle.
    :return: A list of points representing the boustrophedon path within the polygon.
    """
    minx, miny, maxx, maxy = polygon.bounds
    path = []

    y = miny + vision_radius  # Start from the first line inside the polygon
    direction = 1  # 1 for right, -1 for left

    while y <= maxy:  # Ensure the path stays within the turning radius of the top edge
        if direction == 1:
            x_coords = np.arange(minx, maxx + vision_radius * 2, vision_radius * 2)
        else:
            x_coords = np.arange(maxx, minx - vision_radius * 2, -vision_radius * 2)

        line = LineString([(x, y) for x in x_coords])
        intersected = polygon.intersection(line)

        if isinstance(intersected, LineString):
            coords = list(intersected.coords)
            adjusted_coords = adjust_line_segment(coords, turning_radius, direction)
            path.extend(adjusted_coords)
        elif isinstance(intersected, unary_union.__class__):
            for geom in intersected.geoms:
                coords = list(geom.coords)
                adjusted_coords = adjust_line_segment(coords, turning_radius, direction)
                path.extend(adjusted_coords)

        y += vision_radius * 2
        direction *= -1

    return path


def distance(p1, p2, polygon, obstacles):
    """
    Euclidean distance between two points with penalty for crossing obstacles or going outside the polygon.
    :param p1: first point (x, y)
    :param p2: second point (x, y)
    :param polygon: List of points defining the polygonal area.
    :param obstacles: List of lists of points defining obstacles within the polygonal area.
    :return: Euclidean distance with penalty
    """
    line = LineString([p1, p2])
    polygon_shape = Polygon(polygon)
    obstacle_shapes = [Polygon(obstacle) for obstacle in obstacles]
    all_obstacles = unary_union(obstacle_shapes)  # Merge all obstacles into a single shape

    # Initialize penalty
    penalty = 0

    # Check if line goes outside the polygon
    if not polygon_shape.contains(line):
        # Calculate the intersection of the line with the polygon boundary
        intersection_with_polygon = line.intersection(polygon_shape.exterior)
        if intersection_with_polygon.is_empty:
            intersection_with_polygon = line.intersection(polygon_shape)
        if not intersection_with_polygon.is_empty:
            intersection_length = 0
            if intersection_with_polygon.geom_type == 'LineString':
                intersection_length = intersection_with_polygon.length
            elif intersection_with_polygon.geom_type == 'MultiLineString':
                intersection_length = sum(line_segment.length for line_segment in intersection_with_polygon)
            penalty += intersection_length * (polygon_shape.length / 2)

    # Check if line intersects any obstacle
    for obstacle in obstacles:
        obstacle_shape = Polygon(obstacle)
        intersection_with_obstacle = line.intersection(obstacle_shape.exterior)
        if intersection_with_obstacle.is_empty:
            intersection_with_obstacle = line.intersection(obstacle_shape)
        if not intersection_with_obstacle.is_empty:
            intersection_length = 0
            if intersection_with_obstacle.geom_type == 'LineString':
                intersection_length = intersection_with_obstacle.length
            elif intersection_with_obstacle.geom_type == 'MultiLineString':
                intersection_length = sum(line_segment.length for line_segment in intersection_with_obstacle)
            penalty += intersection_length * (polygon_shape.length / 2)
            break  # Break after the first obstacle intersection to avoid double penalty

    euclidean_distance = ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    return euclidean_distance + penalty


def tsp_order(points, polygon, obstacles):
    """
    Solves the TSP using a brute-force approach.
    :param points: list of (x, y) positions
    :return: optimal order of points for TSP
    """
    n = len(points)
    min_distance = float('inf')
    best_order = None

    for order in permutations(range(n)):
        current_distance = 0
        for i in range(n):
            current_distance += distance(points[order[i]], points[order[(i + 1) % n]], polygon, obstacles)
        if current_distance < min_distance:
            min_distance = current_distance
            best_order = order

    return best_order


def aggregate_paths(point_paths, polygon, obstacles):
    """
    Aggregates subpaths into a single path in the optimal order.
    :param point_paths: list of subpaths
    :return: aggregated path
    """
    endpoints = [(path[0], path[-1]) for path in point_paths]
    start_points = [point[0] for point in endpoints]
    end_points = [point[1] for point in endpoints]

    # Solve TSP for the start points
    optimal_order = tsp_order(start_points, polygon, obstacles)

    aggregated_path = []
    for idx in optimal_order:
        aggregated_path.extend(point_paths[idx])

    return aggregated_path


def boustrophedon_solve(polygon, obstacle, vision_radius, turning_radius):
    """
    Solve the path planning problem using the boustrophedon method to cover the polygonal area while avoiding obstacles.
    :param polygon: List of points defining the polygonal area that needs to be covered.
    :param obstacle: List of points defining obstacles within the polygonal area.
    :param vision_radius: Distance ahead the vehicle can "see" or consider for its next move.
    :param turning_radius: Minimum turning radius of the vehicle.
    :return:  Length of the final solution and the solution itself
    """
    min_strip_height = vision_radius * 2
    convex_polygons = boustrophedon_cells(polygon, obstacle, min_strip_height)

    point_paths = []
    for poly in convex_polygons:
        point_path = boustrophedon_path(poly, vision_radius, turning_radius)
        point_paths.append(point_path)

    # final_path = []
    # for path in point_paths:
    #     for point in path:
    #         final_path.append(point[:2])
    #
    # length, final_path_optimised_dubins = compute_dubins_paths_and_length(final_path,
    #                                                                       [i for i in range(len(final_path))],
    #                                                                       turning_radius)
    aggregated_path = aggregate_paths(point_paths, polygon, obstacle)
    order = list(range(len(aggregated_path)))

    length, final_path_optimised_dubins = compute_dubins_paths_and_length(aggregated_path, order, turning_radius)

    return length, final_path_optimised_dubins


def generate_dubins_path(start, end, turning_radius):
    """
    Generate Dubins path between start and end points.
    :param start:
    :param end:
    :param turning_radius:
    :return:
    """
    q0 = (start[0], start[1], start[2])  # (x, y, theta)
    q1 = (end[0], end[1], end[2])  # (x, y, theta)
    path = dubins.shortest_path(q0, q1, turning_radius)
    configurations, _ = path.sample_many(0.1)
    if configurations:  # Ensure configurations is not empty
        return np.array(configurations)[:, :3], path.path_length()  # Return configurations and path length
    else:
        return np.array([]), float('inf')  # Return an empty array and infinite length if configurations is empty


def point_in_polygon(polygon, point):
    """
    Check if a point is in a specified polygon
    :return:
    """
    return polygon.contains(Point(point[0], point[1]))


def distance_point_to_segment(px, py, ax, ay, bx, by):
    """
    Compute the distance from point (px, py) to segment (ax, ay)-(bx, by).
    :return:
    """
    seg_length = np.hypot(bx - ax, by - ay)
    if seg_length == 0:
        return np.hypot(px - ax, py - ay)
    u = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / seg_length ** 2
    u = max(0, min(1, u))
    dx = ax + u * (bx - ax) - px
    dy = ay + u * (by - ay) - py
    return np.hypot(dx, dy)


def find_initial_point(polygon, vision_radius):
    """
    Find the initial point inside the polygon near the edge.
    :param polygon:
    :param vision_radius:
    :return:
    """
    min_x, min_y, max_x, max_y = polygon.bounds
    for x in np.linspace(min_x + vision_radius, max_x - vision_radius, 100):
        for y in np.linspace(min_y + vision_radius, max_y - vision_radius, 100):
            point = np.array([x, y])
            if point_in_polygon(polygon, point):
                edge_point, direction = closest_edge_and_orientation(polygon, point, vision_radius)
                distance_to_edge = np.linalg.norm(point - edge_point)
                if distance_to_edge >= vision_radius:
                    return point[0], point[1], direction
    raise ValueError("Could not find a valid initial point inside the polygon.")


def closest_edge_and_orientation(polygon, point, vision_radius):
    """Find the closest edge to the point and determine its inward orientation to the right."""
    min_distance = float('inf')
    closest_edge = None
    bottom_edge = None

    for i in range(len(polygon.exterior.coords) - 1):
        (ax, ay), (bx, by) = polygon.exterior.coords[i], polygon.exterior.coords[i + 1]
        if ay == by and ay < min(point[1], polygon.centroid.y):
            bottom_edge = ((ax, ay), (bx, by))
        dist = distance_point_to_segment(point[0], point[1], ax, ay, bx, by)
        if dist < min_distance:
            min_distance = dist
            closest_edge = ((ax, ay), (bx, by))

    if bottom_edge:
        closest_edge = bottom_edge

    if closest_edge:
        (ax, ay), (bx, by) = closest_edge
        edge_vector = np.array([bx - ax, by - ay])
        edge_direction = np.arctan2(edge_vector[1], edge_vector[0])

        # Calculate the normal vector
        normal_vector = np.array([-edge_vector[1], edge_vector[0]])
        normal_vector = normal_vector / np.linalg.norm(normal_vector)  # Normalize the vector

        # Ensure normal vector points to the right of the point
        right_point = np.array([point[0] + normal_vector[0] * vision_radius * 1.5,
                                point[1] + normal_vector[1] * vision_radius * 1.5])
        if not polygon.contains(Point(right_point)):
            normal_vector = -normal_vector

        inward_direction = np.arctan2(normal_vector[1], normal_vector[0])
        return np.array([ax, ay]), inward_direction

    raise ValueError("No closest edge found.")


def valid_new_point(polygon, new_point, vehicle_path, vision_radius, recent_path_points):
    """
   Check if a new point is valid: it must be inside the polygon and not too close to any other point in the path,
   except the last point.
   :param polygon: List of points defining the polygonal area within which the vehicle must navigate.
   :param new_point: The new point to be validated as [x, y, theta].
   :param vehicle_path: List of points representing the path the vehicle has taken so far.
   :param vision_radius: Distance ahead the vehicle can "see" or consider for its next move.
   :param recent_path_points: List of recent points in the vehicle's path to avoid backtracking.
   :return: Boolean indicating whether the new point is valid.
   """
    if not point_in_polygon(polygon, new_point):
        return False

    new_point_arr = np.array(new_point[:2])

    for path_point in recent_path_points:
        path_point_arr = np.array(path_point[:2])
        distance = np.linalg.norm(new_point_arr - path_point_arr)
        if distance < 1.8 * vision_radius:
            return False

    return True


def generate_next_point(polygon, current_point, vision_radius, turning_radius, vehicle_path, recent_path_points):
    """
        Generate the next point for the vehicle's path based on the current point, vision radius, turning radius,
        and constraints imposed by the polygon and recent path points.
        :param polygon: List of points defining the polygonal area within which the vehicle must navigate.
        :param current_point: Current position and heading of the vehicle as [x, y, theta].
        :param vision_radius: Distance ahead the vehicle can "see" or consider for its next move.
        :param turning_radius: Minimum turning radius of the vehicle.
        :param vehicle_path: List of points representing the path the vehicle has taken so far.
        :param recent_path_points: List of recent points in the vehicle's path to avoid backtracking.
        :return: Tuple containing the segment of the Dubins path and the next point as ([path_segment], [next_point]).
        """
    # Set the angle increment here
    angle_increment = np.pi / 64  # 11.25 degrees increment

    # Define the min and max angles
    min_angle = -3 * np.pi / 8  # Minimum angle
    # max_angle = 15 * np.pi / 16  # Maximum angle
    max_angle = np.pi - angle_increment  # Maximum angle

    # Define scaling bounds
    max_scaling = 2.8 * turning_radius

    # Calculate the current angle
    current_angle = current_point[2]

    # Calculate the number of directions based on the angle increment
    num_directions = int((max_angle - min_angle) / angle_increment) + 1

    # Generate the directions list
    directions = [min_angle + i * angle_increment for i in range(num_directions)]  # Check within the specified range

    for delta_angle in directions:
        new_angle = current_angle + delta_angle

        distance = max_scaling

        # Compute the new point coordinates
        next_x = current_point[0] + distance * np.cos(new_angle)
        next_y = current_point[1] + distance * np.sin(new_angle)
        next_point = [next_x, next_y, new_angle]

        # Validate forward point
        # if point_in_polygon(polygon, forward_point):
        if valid_new_point(polygon, next_point, vehicle_path, vision_radius, recent_path_points):
            # Calculate the next point

            forward_x = current_point[0] + (distance + vision_radius) * np.cos(new_angle)
            forward_y = current_point[1] + (distance + vision_radius) * np.sin(new_angle)
            forward_point = [forward_x, forward_y, new_angle]

            # Validate next point
            if valid_new_point(polygon, forward_point, vehicle_path, vision_radius, recent_path_points):
                # if point_in_polygon(polygon, next_point):
                dubins_path_segment, _ = generate_dubins_path(current_point, next_point, turning_radius)
                if len(dubins_path_segment) > 0:
                    return dubins_path_segment, next_point

    # Return empty if no valid path segment is found
    return np.array([]), current_point


def dubins_vehicle_simulation(polygon_coords, vision_radius, turning_radius, max_iterations=50000):
    """
    Perform an iterative construction of a solution build using a dubins vehicle simulation
    :param polygon_coords:
    :param vision_radius:
    :param turning_radius:
    :param max_iterations:
    :return:
    """
    polygon = Polygon(polygon_coords)
    vehicle_path = []

    # Find the initial point and orientation
    start = find_initial_point(polygon, vision_radius)
    vehicle_path.append(start)

    # Initialize the recent path points (simplified path)
    recent_path_points = []
    point_path = []

    iterations = 0
    while iterations < max_iterations:
        dubins_path_segment, next_point = generate_next_point(polygon, vehicle_path[-1], vision_radius, turning_radius,
                                                              vehicle_path, list(recent_path_points))
        if len(dubins_path_segment) > 0:
            recent_path_points = deepcopy(vehicle_path)
            point_path.append(next_point)
            vehicle_path.extend(dubins_path_segment)
            # Corrected comparison: extract x and y only from the last points
            if np.linalg.norm(vehicle_path[-1][:2] - np.array(recent_path_points[-1][:2])) < vision_radius * 0.5:
                break  # Break if the vehicle is revisiting the same point
        else:
            break  # Stop if no valid next point is found
        iterations += 1

    while iterations < max_iterations:
        # For 1 last point to ensure that the middle of the polygon is reached
        dubins_path_segment, next_point = generate_next_point(polygon, vehicle_path[-1], vision_radius * 0.7,
                                                              turning_radius,
                                                              vehicle_path, list(recent_path_points))
        if len(dubins_path_segment) > 0:
            recent_path_points = deepcopy(vehicle_path)
            point_path.append(next_point)
            vehicle_path.extend(dubins_path_segment)
        else:
            break  # Stop if no valid next point is found
        iterations += 1

    return_path, _ = generate_dubins_path(vehicle_path[-1], vehicle_path[0], turning_radius)
    vehicle_path.extend(return_path)

    return np.array(vehicle_path), point_path


def dubins_simulation_solve(polygon, obstacles, vision_radius, turning_radius, cpp_executable_path):
    """
    Method that build an iterative solution then optimise it using a simple local search
    :param cpp_executable_path: for the simple local search performed on the iterative solution
    :param polygon: area to cover
    :param obstacles:
    :param vision_radius:
    :param turning_radius:
    :return: length of the final solution and the solution itself
    """
    # Split area in convex cells
    convex_polygons = boustrophedon_cells(polygon, obstacles, vision_radius * 2)

    vehicle_paths = []
    point_paths = []
    # Perform iterative resolution on ach cell
    for poly in convex_polygons:
        vehicle_path, point_path = dubins_vehicle_simulation(poly, vision_radius, turning_radius)
        vehicle_paths.append(vehicle_path)
        point_paths.append(point_path)

    # final_path = []
    # for path in point_paths:
    #     for point in path:
    #         final_path.append(point[:2])

    #

    aggregated_path = aggregate_paths(point_paths, polygon, obstacles)
    order = list(range(len(aggregated_path)))

    # Combine all path into a global optimised solution using simple local search
    # opti_path = cpp_opti([i for i in range(len(final_path))], np.full((1, 1), 0), 5, cpp_executable_path,
    #                      points=final_path, turning_radius=turning_radius)
    opti_path = cpp_opti(order, np.full((1, 1), 0), 5, cpp_executable_path,
                                               points=aggregated_path, turning_radius=turning_radius)

    final_path_optimised = []
    for index in opti_path:
        # final_path_optimised.append(final_path[index])
        final_path_optimised.append(order[index])

    # length, final_path_optimised_dubins = compute_dubins_paths_and_length(final_path, opti_path, turning_radius)
    length, final_path_optimised_dubins = compute_dubins_paths_and_length(aggregated_path, opti_path, turning_radius)

    return length, final_path_optimised_dubins
