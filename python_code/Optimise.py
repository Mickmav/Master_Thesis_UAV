import random
import math
import subprocess
import json
import dubins
import numpy as np


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
