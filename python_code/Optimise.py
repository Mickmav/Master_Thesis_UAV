import random
import math
import subprocess
import json


def cpp_opti(path, matrix, type_of_optimisation, cpp_executable_path):
    """
    Apply a given cpp program to perform the optimisations functions more optimally
    :param path: a given starting solution
    :param matrix: a matrix of distance between the points
    :param type_of_optimisation: the type of optimisation need (0 = SA / 1 = GA / 2 = SLS)
    :param cpp_executable_path: path of the cpp compiled program
    :return: The final solution
    """
    # Prepare input data as a dictionary
    matrix = matrix.tolist()
    input_data = {"list": path, "matrix": matrix, "type": type_of_optimisation}

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

        print("Received output list:", calculate_cost(output_list, matrix), output_list)
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


def optimise(path, matrix, type_of_optimisation=0):
    """
     Apply a given python algorithm to perform the optimisations functions
    :param path: a given starting solution
    :param matrix: a matrix of distance between the points
    :param type_of_optimisation: the type of optimisation need (0 = SA / 1 = GA / 2 = SLS)
    :return: The final solution
    """
    new_sol = []
    if type_of_optimisation == 0:
        new_sol = simulated_annealing(path, matrix, 900, 0.009)
    if type_of_optimisation == 1:
        new_sol = genetic_algorithm(len(path), matrix, 100)
    if type_of_optimisation == 2:
        new_sol = simple_local_search(path, matrix)

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


def simulated_annealing(initial_solution, matrix, temperature=900, min_temperature=0.009):
    """
    Perform a simulated algorithm, that perform random modification on a solution and has a probability
    (based on a variable called temperature) to accept worse solution to extend the search scope
    :param initial_solution:
    :param matrix: matrix of distance between points
    :param temperature:
    :param min_temperature: when temperature is to small stop the process
    :return: the best solution obtained
    """
    def acceptance_probability(cost_diff, temperature):
        """
        Probability to accept worsening solution
        :param cost_diff:
        :param temperature:
        :return:
        """
        return math.exp(-cost_diff / temperature)
    # Initialize the process
    cooling_rate = compute_cooling_rate(len(initial_solution))
    current_solution = initial_solution
    best_solution = initial_solution
    best_cost = calculate_cost(best_solution, matrix)

    i = 0
    while temperature > min_temperature:
        # Modify the solution
        new_solution = random_exchange(current_solution)  # Small random perturbation

        current_cost = calculate_cost(current_solution, matrix)
        new_cost = calculate_cost(new_solution, matrix)

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


def generate_random_solution(nb_points):
    """
    Generate a completely random solution
    :param nb_points: size of a solution
    :return: the random solution
    """
    generated_solution = []
    choices = [i for i in range(nb_points)]
    for i in range(nb_points):
        new_val = random.choice(choices)
        generated_solution.append(new_val)
        if len(choices) != 1:
            choices.remove(new_val)

    return generated_solution


def find_best(initial_population, matrix, size_returning_list):
    """
    Sort a list based on the cost function, then extract the "n best" solution
    :param initial_population: starting group of solution
    :param matrix: matrix of distance between the point
    :param size_returning_list: number of solution to return
    :return: the n better solution in a sorted list
    """
    def custom_key(solution):
        return calculate_cost(solution, matrix)

    initial_population = sorted(initial_population, key=custom_key)
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


def mutate(solutions, matrix):
    """
    Perform a random permutation on all the solutions
    :param solutions: list of all the solution
    :param matrix: matrix of distance between the points
    :return: a modified version of the received solution
    """
    new_solutions = []
    for solution in solutions:
        cost = calculate_cost(solution, matrix)
        new_solution = random_exchange(solution)
        new_cost = calculate_cost(new_solution, matrix)
        stop = 0
        while new_cost > cost and stop < 2:
            stop += 1
            new_solution = random_exchange(solution)
            new_cost = calculate_cost(new_solution, matrix)
        if new_cost < cost:
            solution = new_solution
        new_solutions.append(solution)
    return new_solutions


def genetic_algorithm(nb_points, matrix, pop_size):
    """
    Perform a genetic algorithm with mutation, cross-over with survival type of selection
    :param nb_points: number of point in the solution
    :param matrix: matrix of distance
    :param pop_size: size of the population of solution
    :return: the best solution obtained
    """
    population = []
    for i in range(pop_size):
        population.append(generate_random_solution(nb_points))
    best_solution = find_best(population, matrix, 1)[0]
    best_cost_ever = calculate_cost(best_solution, matrix)

    repetition = 0
    i = 0
    while repetition < nb_points + 10:
        best_pop = find_best(population, matrix, int(pop_size / 10))
        cross_over_pop = cross_over(best_pop, population)
        mutated_pop = mutate(population, matrix)
        mutated_cross = mutate(cross_over_pop, matrix)
        population = population + cross_over_pop + mutated_pop + mutated_cross
        population = find_best([list(point) for point in set(tuple(solution) for solution in population)], matrix,
                               pop_size)
        new_cost = calculate_cost(population[0], matrix)
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


def simple_search(solution, matrix):
    """
    Search using 2-exchange method with "best improvement" pivoting rule
    :param solution:
    :param matrix: matrix of distance
    :return: upgrade version of the input solution
    """
    nb_points = len(solution)
    best_cost = calculate_cost(solution, matrix)
    best_sol = solution
    for point in range(nb_points - 1):
        for second_point in range(point + 1, nb_points):
            part1 = solution[:point]
            part2 = solution[point:second_point + 1]
            part3 = solution[second_point + 1:]

            new_sol = part1 + part2[::-1] + part3
            new_cost = calculate_cost(new_sol, matrix)
            if new_cost < best_cost:
                best_sol = new_sol
                best_cost = new_cost
    return best_sol


def simple_local_search(solution, matrix):
    """
    Perform a simple search on a solution until no upgrade is found
    :param solution:
    :param matrix: matrix of distance
    :return: the best solution obtained
    """
    solution_cost = calculate_cost(solution, matrix)
    new_cost = 0
    i = 0
    while solution_cost != new_cost:
        solution_cost = new_cost
        solution = simple_search(solution, matrix)
        new_cost = calculate_cost(solution, matrix)

        i += 1
        if i == 20:
            i = 0
            print(solution_cost, solution)

    return solution
