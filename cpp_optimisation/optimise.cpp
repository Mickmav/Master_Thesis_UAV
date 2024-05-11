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
using namespace std;

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

/// Apply a given python algorithm to perform the optimisations functions
/// \param path a given starting solution
/// \param matrix a matrix of distance between the points
/// \param type_of_optimisation the type of optimisation need (0 = SA / 1 = GA / 2 = SLS)
/// \return The final solution
std::vector<int> optimise(std::vector<int>& path, std::vector<std::vector<int>>& matrix, int type_of_optimisation = 0) {
    std::vector<int> new_sol;

    if (type_of_optimisation == 0) {
        new_sol = simulated_annealing(path, matrix, 900, 0.009);
    } else if (type_of_optimisation == 1) {
        new_sol = genetic_algorithm(path.size(), matrix, 100);
    } else if (type_of_optimisation == 2) {
        new_sol = simple_local_search(path, matrix);
    }

//    std::cout << calculate_cost(new_sol, matrix) << " ";
//
//    for (long unsigned int i = 0; i < new_sol.size(); ++i) {
//        std::cout << new_sol[i] << " ";
//    }
//
//    std::cout << std::endl;

    return new_sol;
}


