#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <chrono>
#include "optimise.h"
#include "dubins.h"
using namespace std;


#include <fstream>  // For file I/O

typedef vector<int> Chromosome;


///   Cost function of the different algorithms
/// \param path a given solution
/// \param matrix  matrix of distance between the points
/// \return  cost of the solution
int calculate_cost(const std::vector<int>& path, const std::vector<std::vector<int>>& matrix) {
    int cost = 0;
    for (size_t index = 0; index < path.size(); ++index) {
        cost += matrix[path[index]][path[(index + 1) % path.size()]];
    }
    return cost;
}

// Function to normalize a 2D vector
void normalize(vector<double>& vec) {
    double norm = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
    if (norm != 0) {
        vec[0] /= norm;
        vec[1] /= norm;
    }
}

// Function to compute headings between points
vector<double> compute_headings_2(const vector<pair<double, double>>& points, const vector<int>& order) {
    vector<double> headings;
    int size = order.size();

    for (int i = 0; i < size; ++i) {
        auto p0 = points[order[(i - 1 + size) % size]];
        auto p1 = points[order[i]];
        auto p2 = points[order[(i + 1) % size]];

        // Vectors from p0 to p1 and p1 to p2
        vector<double> v1 = {p1.first - p0.first, p1.second - p0.second};
        vector<double> v2 = {p2.first - p1.first, p2.second - p1.second};

        // Normalize vectors
        normalize(v1);
        normalize(v2);

        // Calculate the sum of vectors
        vector<double> v_sum = {v1[0] + v2[0], v1[1] + v2[1]};

        double heading;
        if (sqrt(v_sum[0] * v_sum[0] + v_sum[1] * v_sum[1]) == 0) {
            // Handle the case where v1 and v2 are exact opposites
            heading = atan2(v1[1], v1[0]);  // Arbitrary choice: use the direction of v1
        } else {
            // Calculate the bisector vector
            normalize(v_sum);
            // Calculate the heading angle
            heading = atan2(v_sum[1], v_sum[0]);
        }

        headings.push_back(heading);
    }

    return headings;
}

// Function to compute total Dubins path length
double compute_total_dubins_path_length(const vector<pair<double, double>>& points, const vector<int>& order,
                                        double turning_radius) {
    vector<double> headings = compute_headings_2(points, order);
    double total_length = 0.0;

    for (size_t i = 0; i < order.size(); ++i) {
        int idx_start = order[i];
        int idx_end = order[(i + 1) % order.size()];

        // Start and end configurations with computed headings
        double start_point[3] = {points[idx_start].first, points[idx_start].second, headings[i]};
        double end_heading = (i + 1 < headings.size()) ? headings[i + 1] : headings[0];
        double end_point[3] = {points[idx_end].first, points[idx_end].second, end_heading};

        // Dubins path computation
        DubinsPath path;
        dubins_shortest_path(&path, start_point, end_point, turning_radius);
        double path_length = dubins_path_length(&path);

        total_length += path_length;
    }

    return total_length;
}

/// Compute a cooling rate based of the size of a solution
/// \param size size of a solution
/// \return cooling rate
double compute_cooling_rate(int size) {
    std::string cooling_rate_str = "0." + std::string(log(size), '9');
    return std::stod(cooling_rate_str);
}

/// Perform a random 2-exchange in a given solution
/// \param solution
/// \return a modified solution
std::vector<int> random_exchange(const std::vector<int>& solution) {
    int start = std::rand() % solution.size();
    std::vector<int> possible_choice = solution;
    possible_choice.erase(possible_choice.begin() + start);
    int end = possible_choice[std::rand() % possible_choice.size()];
    if (start > end) {
        std::swap(start, end);
    }
    std::vector<int> part1(solution.begin(), solution.begin() + start);
    std::vector<int> part2(solution.begin() + start, solution.begin() + end + 1);
    std::vector<int> part3(solution.begin() + end + 1, solution.end());

    std::vector<int> new_sol(part1);
    new_sol.insert(new_sol.end(), part2.rbegin(), part2.rend());
    new_sol.insert(new_sol.end(), part3.begin(), part3.end());

    return new_sol;
}

/// Perform a simulated algorithm, that perform random modification on a solution and has a probability
/// (based on a variable called temperature) to accept worse solution to extend the search scope
/// \param initial_solution
/// \param matrix matrix of distance between points
/// \param temperature
/// \param min_temperature when temperature is to small stop the process
/// \return the best solution obtained
std::vector<int> simulated_annealing(const std::vector<int>& initial_solution, const std::vector<std::vector<int>>& matrix,
                                     double temperature = 900, double min_temperature = 0.009) {
    // Initialize the process
    double cooling_rate = compute_cooling_rate(initial_solution.size());
    std::vector<int> current_solution = initial_solution;
    std::vector<int> best_solution = initial_solution;
    int best_cost = calculate_cost(best_solution, matrix);

//    int i = 0;
    while (temperature > min_temperature) {
        // Modify the solution
        std::vector<int> new_solution = random_exchange(current_solution);

        int current_cost = calculate_cost(current_solution, matrix);
        int new_cost = calculate_cost(new_solution, matrix);
        // Verify the solution and accept if conditions are met
        if (new_cost < current_cost || std::rand() / double(RAND_MAX) < std::exp(-double(new_cost - current_cost) / temperature)) {
            current_solution = new_solution;
        }
        // Change the best solution when a better solution is found
        if (current_cost < best_cost) {
            best_solution = current_solution;
            best_cost = current_cost;
        }
        // Update temperature
        temperature *= cooling_rate;
//        i++;
//        if (i >= 200000) {
//            i = 0;
//            std::cout << temperature << " " << best_cost << " ";
//            for (int val : best_solution) {
//                std::cout << val << " ";
//            }
//            std::cout << std::endl;
//        }
    }
    return best_solution;
}

/// Perform a simulated annealing algorithm that performs random modifications on a solution and has a probability
/// (based on a variable called temperature) to accept a worse solution to extend the search scope
/// \param initial_solution
/// \param points the x,y positions of all the points
/// \param turning_radius the turning radius for Dubins path computation
/// \param temperature
/// \param min_temperature when temperature is too small stop the process
/// \return the best solution obtained
std::vector<int> simulated_annealing_dubins(const std::vector<int>& initial_solution, const std::vector<std::pair<double, double>>& points,
                                     double turning_radius, double temperature = 900, double min_temperature = 0.009) {
    // Initialize the process
    double cooling_rate = compute_cooling_rate(initial_solution.size());  // Assuming a fixed cooling rate for simplicity
    std::vector<int> current_solution = initial_solution;
    std::vector<int> best_solution = initial_solution;
    double best_cost = compute_total_dubins_path_length(points, best_solution, turning_radius);
    while (temperature > min_temperature) {
        // Modify the solution
        std::vector<int> new_solution = random_exchange(current_solution);

        double current_cost = compute_total_dubins_path_length(points, current_solution, turning_radius);
        double new_cost = compute_total_dubins_path_length(points, new_solution, turning_radius);

        // Verify the solution and accept if conditions are met
        if (new_cost < current_cost || std::rand() / double(RAND_MAX) < std::exp(-double(new_cost - current_cost) / temperature)) {
            current_solution = new_solution;
        }

        // Change the best solution when a better solution is found
        if (current_cost < best_cost) {
            best_solution = current_solution;
            best_cost = current_cost;
        }

        // Update temperature
        temperature *= cooling_rate;
    }

    return best_solution;
}

/// Generate a completely random solution
/// \param nb_points size of a solution
/// \return the random solution
std::vector<int> generate_random_solution(int nb_points) {
    std::vector<int> generated_solution;
    std::vector<int> choices(nb_points);
    for (int i = 0; i < nb_points; ++i) {
        choices[i] = i;
    }

    for (int i = 0; i < nb_points; ++i) {
        int new_val = choices[std::rand() % choices.size()];
        generated_solution.push_back(new_val);

        if (choices.size() != 1) {
            choices.erase(std::remove(choices.begin(), choices.end(), new_val), choices.end());
        }
    }

    return generated_solution;
}

/// Sort a list based on the cost function, then extract the "n best" solution
/// \param initial_population starting group of solution
/// \param matrix matrix of distance between the point
/// \param size_returning_list number of solution to return
/// \return the n better solution in a sorted list
std::vector<std::vector<int>> find_best(std::vector<std::vector<int>>& initial_population,
                                        const std::vector<std::vector<int>>& matrix,
                                        int size_returning_list) {
    auto custom_key = [&matrix](const std::vector<int>& solution) {
        return calculate_cost(solution, matrix);
    };

    std::sort(initial_population.begin(), initial_population.end(), [&](const std::vector<int>& a, const std::vector<int>& b) {
        return custom_key(a) < custom_key(b);
    });

    std::vector<std::vector<int>> best_solutions(initial_population.begin(), initial_population.begin() + size_returning_list);
    return best_solutions;
}

/// Perform a mix between the bests solutions and all the solutions
/// \param best Group of the best solutions
/// \param solutions all the solutions
/// \return list of the new solutions
std::vector<std::vector<int>> cross_over(const std::vector<std::vector<int>>& best,
                                         const std::vector<std::vector<int>>& solutions) {
    std::vector<std::vector<int>> new_pop;
    int size_sol = solutions[0].size();
    // Take a solution as parent 2
    for (const std::vector<int>& parent2 : solutions) {
        // Choose one of the best solution as parent 1
        const std::vector<int>& parent1 = best[std::rand() % best.size()];

        int start = std::rand() % (size_sol - 1);
        int end = start + 1 + std::rand() % (size_sol - start - 1);

        // Choose interval for the cross-over
        std::vector<int> child1(size_sol, -1);
        std::vector<int> child2(size_sol, -1);

        // Cross the 2 solution
        for (int i = start; i <= end; ++i) {
            child1[i] = parent1[i];
            child2[i] = parent2[i];
        }

        for (int i = 0; i < size_sol; ++i) {
            if (std::find(child1.begin(), child1.end(), parent2[i]) == child1.end()) {
                auto it = std::find(child1.begin(), child1.end(), -1);
                *it = parent2[i];
            }

            if (std::find(child2.begin(), child2.end(), parent1[i]) == child2.end()) {
                auto it = std::find(child2.begin(), child2.end(), -1);
                *it = parent1[i];
            }
        }

        new_pop.push_back(child1);
        new_pop.push_back(child2);
    }

    return new_pop;
}

/// Perform a random permutation on all the solutions
/// \param solutions list of all the solution
/// \param matrix matrix of distance between the points
/// \return a modified version of the received solution
std::vector<std::vector<int>> mutate(const std::vector<std::vector<int>>& solutions, const std::vector<std::vector<int>>& matrix) {
    const int mutation_try_limit = 2;
    std::vector<std::vector<int>> new_solutions;
    for (const std::vector<int>& solution : solutions) {
        int cost = calculate_cost(solution, matrix);
        std::vector<int> new_solution = random_exchange(solution);
        int new_cost = calculate_cost(new_solution, matrix);
        int stop = 0;

        while ( new_cost > cost && stop < mutation_try_limit) {
            stop++;
            new_solution = random_exchange(solution);
            new_cost = calculate_cost(new_solution, matrix);
        }
        if (new_cost > cost){
            new_solution = solution;
        }
        new_solutions.push_back(new_solution);
    }

    return new_solutions;
}

/// Perform a genetic algorithm with mutation, cross-over with survival type of selection
/// \param nb_points number of point in the solution
/// \param matrix matrix of distance
/// \param pop_size size of the population of solution
/// \return the best solution obtained
std::vector<int> genetic_algorithm(int nb_points, const std::vector<std::vector<int>>& matrix, int pop_size) {
    const double best_solution_criterion = 0.1;
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::vector<std::vector<int>> population;
    for (int i = 0; i < pop_size; ++i) {
        population.push_back(generate_random_solution(nb_points));
    }
    population = find_best(population, matrix, pop_size);
    std::vector<int> best_solution = population[0];
    int best_cost_ever = calculate_cost(best_solution, matrix);
    int new_cost;
    int repetition = 0;

//    int i = 0;
    while (repetition < nb_points + 10) {
        std::vector<std::vector<int>> best_pop = find_best(population, matrix,
                                                           pop_size * best_solution_criterion);
        std::vector<std::vector<int>> cross_over_pop = cross_over(best_pop, population);
        std::vector<std::vector<int>> mutated_pop = mutate(population, matrix);
        std::vector<std::vector<int>> mutated_cross = mutate(cross_over_pop, matrix);

        population.insert(population.end(), cross_over_pop.begin(), cross_over_pop.end());
        population.insert(population.end(), mutated_pop.begin(), mutated_pop.end());
        population.insert(population.end(), mutated_cross.begin(), mutated_cross.end());
        population = find_best(population, matrix, pop_size);
        best_solution = population[0];
        new_cost = calculate_cost(best_solution, matrix);

        if (new_cost == best_cost_ever) {
            repetition++;
        } else {
            repetition = 0;
        }

        best_cost_ever = new_cost;

//        i++;
//        if (i >= 50) {
//            i = 0;
//            std::cout << i << " " << best_cost_ever << std::endl;
//        }
    }
    return best_solution;
}

/// Sort a list based on the cost function, then extract the "n best" solution
/// \param initial_population starting group of solution
/// \param points the x,y positions of all the points
/// \param size_returning_list number of solution to return
/// \param turning_radius the turning radius for Dubins path computation
/// \return the n better solution in a sorted list
std::vector<std::vector<int>> find_best_dubins(std::vector<std::vector<int>>& initial_population,
                                               const std::vector<std::pair<double, double>>& points,
                                               int size_returning_list,
                                               double turning_radius) {
    auto custom_key = [&points, turning_radius](const std::vector<int>& solution) {
        return compute_total_dubins_path_length(points, solution, turning_radius);
    };

    std::sort(initial_population.begin(), initial_population.end(), [&](const std::vector<int>& a, const std::vector<int>& b) {
        return custom_key(a) < custom_key(b);
    });

    std::vector<std::vector<int>> best_solutions(initial_population.begin(), initial_population.begin() + size_returning_list);
    return best_solutions;
}

/// Perform a random permutation on all the solutions
/// \param solutions list of all the solution
/// \param points the x,y positions of all the points
/// \param turning_radius the turning radius for Dubins path computation
/// \return a modified version of the received solution
std::vector<std::vector<int>> mutate_dubins(const std::vector<std::vector<int>>& solutions,
                                            const std::vector<std::pair<double, double>>& points,
                                            double turning_radius) {
    const int mutation_try_limit = 2;
    std::vector<std::vector<int>> new_solutions;
    for (const std::vector<int>& solution : solutions) {
        double cost = compute_total_dubins_path_length(points, solution, turning_radius);
        std::vector<int> new_solution = random_exchange(solution);
        double new_cost = compute_total_dubins_path_length(points, new_solution, turning_radius);
        int stop = 0;

        while (new_cost > cost && stop < mutation_try_limit) {
            stop++;
            new_solution = random_exchange(solution);
            new_cost = compute_total_dubins_path_length(points, new_solution, turning_radius);
        }
        if (new_cost > cost) {
            new_solution = solution;
        }
        new_solutions.push_back(new_solution);
    }

    return new_solutions;
}

/// Perform a genetic algorithm with mutation, cross-over with survival type of selection
/// \param nb_points number of point in the solution
/// \param points the x,y positions of all the points
/// \param pop_size size of the population of solution
/// \param turning_radius the turning radius for Dubins path computation
/// \return the best solution obtained
std::vector<int> genetic_algorithm_dubins(int nb_points,
                                          const std::vector<std::pair<double, double>>& points,
                                          int pop_size,
                                          double turning_radius) {
    const double best_solution_criterion = 0.1;
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::vector<std::vector<int>> population;
    for (int i = 0; i < pop_size; ++i) {
        population.push_back(generate_random_solution(nb_points));
    }
    population = find_best_dubins(population, points, pop_size, turning_radius);
    std::vector<int> best_solution = population[0];
    double best_cost_ever = compute_total_dubins_path_length(points, best_solution, turning_radius);
    double new_cost;
    int repetition = 0;

    while (repetition < nb_points + 10) {
        std::vector<std::vector<int>> best_pop = find_best_dubins(population, points,
                                                                  pop_size * best_solution_criterion, turning_radius);
        std::vector<std::vector<int>> cross_over_pop = cross_over(best_pop, population);
        std::vector<std::vector<int>> mutated_pop = mutate_dubins(population, points, turning_radius);
        std::vector<std::vector<int>> mutated_cross = mutate_dubins(cross_over_pop, points, turning_radius);

        population.insert(population.end(), cross_over_pop.begin(), cross_over_pop.end());
        population.insert(population.end(), mutated_pop.begin(), mutated_pop.end());
        population.insert(population.end(), mutated_cross.begin(), mutated_cross.end());
        population = find_best_dubins(population, points, pop_size, turning_radius);
        best_solution = population[0];
        new_cost = compute_total_dubins_path_length(points, best_solution, turning_radius);

        if (new_cost == best_cost_ever) {
            repetition++;
        } else {
            repetition = 0;
        }

        best_cost_ever = new_cost;
    }
    return best_solution;
}

/// Search using 2-exchange method with "best improvement" pivoting rule
/// \param solution
/// \param matrix matrix of distance
/// \return upgrade version of the input solution
std::vector<int> simple_search(const std::vector<int>& solution, const std::vector<std::vector<int>>& matrix) {
    int nb_points = solution.size();
    std::vector<int> best_sol = solution;
    int best_cost = calculate_cost(best_sol, matrix);
    int new_cost;

    for (int point = 0; point < nb_points - 1; point++) {
        for (int second_point = point + 1; second_point < nb_points; second_point++) {
            std::vector<int> part1(solution.begin(), solution.begin() + point);
            std::vector<int> part2(solution.begin() + point, solution.begin() + second_point + 1);
            std::vector<int> part3(solution.begin() + second_point + 1, solution.end());

            std::vector<int> new_sol(part1);
            new_sol.insert(new_sol.end(), part2.rbegin(), part2.rend());
            new_sol.insert(new_sol.end(), part3.begin(), part3.end());

            new_cost = calculate_cost(new_sol, matrix);

            if (new_cost < best_cost) {
                best_sol = new_sol;
                best_cost = new_cost;
            }
        }
    }
    return best_sol;
}

/// Perform a simple search on a solution until no upgrade is found
/// \param solution
/// \param matrix matrix of distance
/// \return the best solution obtained
std::vector<int> simple_local_search(const std::vector<int>& solution, const std::vector<std::vector<int>>& matrix) {
    std::vector<int> returning_solution = solution;
    int solution_cost = calculate_cost(solution, matrix);
    int new_cost = 0;

//    int i = 0;
    while ( new_cost != solution_cost) {
        solution_cost = new_cost;
        returning_solution = simple_search(returning_solution, matrix);
        new_cost = calculate_cost(returning_solution, matrix);

//        i++;
//        if (i == 20) {
//            i = 0;
//            std::cout << solution_cost << " ";
//            for (int val : returning_solution) {
//                std::cout << val << " ";
//            }
//            std::cout << std::endl;
//        }

    }
    return returning_solution;
}

/// Search using 2-exchange method with "best improvement" pivoting rule
/// \param solution
/// \param points the x,y positions of all the points
/// \param turning_radius the turning radius for Dubins path computation
/// \return upgrade version of the input solution
vector<int> simple_search_dubins(const vector<int>& solution, const vector<pair<double, double>>& points, double turning_radius) {
    int nb_points = solution.size();
    vector<int> best_sol = solution;
    double best_cost = compute_total_dubins_path_length(points, best_sol, turning_radius);
    double new_cost;

    for (int point = 0; point < nb_points - 1; point++) {
        for (int second_point = point + 1; second_point < nb_points; second_point++) {
            vector<int> part1(solution.begin(), solution.begin() + point);
            vector<int> part2(solution.begin() + point, solution.begin() + second_point + 1);
            vector<int> part3(solution.begin() + second_point + 1, solution.end());

            vector<int> new_sol(part1);
            new_sol.insert(new_sol.end(), part2.rbegin(), part2.rend());
            new_sol.insert(new_sol.end(), part3.begin(), part3.end());

            new_cost = compute_total_dubins_path_length(points, new_sol, turning_radius);

            if (new_cost < best_cost) {
                best_sol = new_sol;
                best_cost = new_cost;
            }
        }
    }
    return best_sol;
}

/// Perform a simple search on a solution until no upgrade is found
/// \param solution
/// \param points the x,y positions of all the points
/// \param turning_radius the turning radius for Dubins path computation
/// \return the best solution obtained
std::vector<int> simple_local_search_dubins(const std::vector<int>& solution,
                                            const std::vector<std::pair<double, double>>& points,
                                            double turning_radius) {
    std::vector<int> returning_solution = solution;
    double solution_cost = compute_total_dubins_path_length(points, solution, turning_radius);
    double new_cost = 0;

    while (solution_cost != new_cost) {
        solution_cost = new_cost;
        returning_solution = simple_search_dubins(returning_solution, points, turning_radius);
        new_cost = compute_total_dubins_path_length(points, returning_solution, turning_radius);
    }
    return returning_solution;
}

/// Apply a given python algorithm to perform the optimisations functions
/// \param path a given starting solution
/// \param matrix a matrix of distance between the points
/// \param type_of_optimisation the type of optimisation need (0 = SA / 1 = GA / 2 = SLS)
/// \return The final solution
std::vector<int> optimise(std::vector<int>& path, std::vector<std::vector<int>>& matrix, std::vector<std::pair<double, double>>& points, int type_of_optimisation = 0, int turning_radius = 0) {
    std::vector<int> new_sol;

    if (type_of_optimisation == 0) {
        new_sol = simulated_annealing(path, matrix, 900, 0.009);
    } else if (type_of_optimisation == 1) {
        new_sol = genetic_algorithm(path.size(), matrix, 100);
    } else if (type_of_optimisation == 2) {
        new_sol = simple_local_search(path, matrix);
    } else if (type_of_optimisation == 3) {
        new_sol = simulated_annealing_dubins(path, points, turning_radius);
    } else if (type_of_optimisation == 4) {
        new_sol = genetic_algorithm_dubins(path.size(), points, 100, turning_radius);
    } else if (type_of_optimisation == 5) {
        new_sol = simple_local_search_dubins(path, points, turning_radius);
            // Create and open a file stream
        std::ofstream output_file("output.txt", std::ios::app);

        if (output_file.is_open()) {
            output_file << "yo" << std::endl;

            // Close the file stream
            output_file.close();
        }
    }

//    // Create and open a file stream
//    std::ofstream output_file("output.txt", std::ios::app);
//
//    if (output_file.is_open()) {
//        int length = compute_total_dubins_path_length(points, new_sol, turning_radius);
//        output_file << length << std::endl;
//
//        // Close the file stream
//        output_file.close();
//    }

//    std::cout << calculate_cost(new_sol, matrix) << " ";
//
//    for (long unsigned int i = 0; i < new_sol.size(); ++i) {
//        std::cout << new_sol[i] << " ";
//    }
//
//    std::cout << std::endl;

    return new_sol;
}


