import argparse
import os
import shutil
import time
# import numpy as np

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from shapely.geometry import Polygon
from concorde.problem import Problem
from concorde.concorde import Concorde
import fast_tsp
from tsp_solver.greedy import solve_tsp
from python_tsp.heuristics import solve_tsp_simulated_annealing
from python_tsp.heuristics import solve_tsp_local_search

from Optimise import optimise, cpp_opti, compute_dubins_paths_and_length, optimise_dubins
from Utils import create_dist_matrix, generate_points_inside_polygon, compute_true_distance, read_vertices, \
    read_config, create_grid, data_linewidth_plot, convert_vertices, convert_vertices_with_ref, \
    convert_relative_to_coordinates, create_pseudo_circle, read_many_vertices


def solve_concorde(matrix):
    print(matrix)
    problem_solver = Problem.from_matrix(matrix)
    solver = Concorde()
    solution = solver.solve(problem_solver)
    # solution = run_concorde(problem_solver)
    print(f'Optimal tour: {solution.tour}')
    return solution.tour


def read_input():
    parser = argparse.ArgumentParser(description="Description of your program")
    # This input will impact how the algorithm will proceed.
    # Either manually or by directly using some specified file in a directory
    parser.add_argument("--manual", help="Do you want manual configurations (y/n)")

    # This parameter impact the nature of the problem to solve (either for Fixed-wings UAV or multi-copter)
    parser.add_argument("--problem", type=int, default=-5,
                        help="The nature of the problem to solve (Fixed-wings UAV or multi-copter)"
                             "\n 0 = Fixed-wings UAV / 1 =  multi-copter")

    # This parameter impact the specific turning radius of the fixed-wings UAV (only for Fixed-wings UAV)
    parser.add_argument("--turning_radius", type=int, default=-5,
                        help="The specific turning radius of the fixed-wings UAV (only useful in the fixed-wings "
                             "problem)")

    # This parameter let the user choose the area he wants to cover
    parser.add_argument("--polygon", type=int, default=-5,
                        help="The polygon you want to cover ( value between 0 and 5)")

    # This parameter let the user choose the obstacles he wants in his instance
    parser.add_argument("--obstacles", type=int, default=-5, help="The obstacles you want to use (-1 no obstacles and "
                                                                  "0 some obstacles)")

    # This parameter impact the algorithm used to solve the TSP instance
    # 0 = SA / 1 = GA / 2 = SLS / 3 = concorde/ 4 = fast_tsp / 5 = tsp_solver2
    # / 6 = solve_tsp_simulated_annealing/ 7 = solve_tsp_local_search
    parser.add_argument("--algo", type=int, default=-5, help="The optimisation algorithm you want to use (values "
                                                             "depend of the problem you want to solve)"
                                                             "\n Problem = 1: 0 = SA / 1 = GA / 2 = SLS / 3 = concorde/"
                                                             "4 = fast_tsp / 5 = tsp_solver2 / "
                                                             "6 = solve_tsp_simulated_annealing /"
                                                             " 7 = solve_tsp_local_search"
                                                             "\n Problem = 0 : 0 = SA / 1 = GA / 2 = SLS /"
                                                             " 3 = tsp_to_dubins")

    # This parameter impact the coverage radius of the UAV
    parser.add_argument("--radius", type=int, default=-5,
                        help="The radius of the area covered by your UAV (value depend on the polygon you choose)")

    args = parser.parse_args()
    if not (args.manual == "y" or args.manual == "yes" or args.manual == "n" or args.manual == "no"):
        print("--manual has only values y, yes, n, no")
        print("Usage :"
              "\n -If not manual : python3 main.py --manual n  (all other argument are not read)"
              "\n -If manual:"
              "\n multi-copter: python3 main.py --manual y --problem 1 --polygon 0 --obstacles -1 --algo 3 --radius 10"
              "\n fixed-wings: python3 main.py --manual y --problem 0 --turning_radius 10 --polygon 0 --obstacles -1 "
              "--algo 3 --radius 10")
        exit(1)

    return args.manual, args.problem, args.polygon, args.obstacles, args.algo, args.radius, args.turning_radius


# Different path for files or directories
cpp_executable_path = "../cpp_optimisation/cpp_program"
save_directory = "../results/"
input_directory = "../configuration/"

if __name__ == '__main__':
    #  --------------------- Retrieve inputs ---------------------------

    manual_input, problem, choice_of_vertex, choice_of_obstacles, type_of_optimisation, radius, turning_radius =\
        read_input()

    # Get the current date and time
    current_time = time.localtime()
    # Format the timestamp as per your requirements
    start_timestamp = time.strftime("%Y%m%d%H%M%S%z", current_time)

    # Record the start time
    global_start_time = time.time()

    # This parameter as an impact on the generation of the different point inside the polygone
    # The systematic way is faster and more suited for the resolution of the SLS algorithm
    # The random way is slower but allow a better coverage of the polygon
    point_generation = "systematic"  # "systematic" / "random"

    # This parameter is used to decide if the optimisation will be proceeded using the c++ or python functions
    method = "python"
    # cpp / python / concorde / fast_tsp / tsp_solver2 / solve_tsp_simulated_annealing / solve_tsp_local_search

    earth_coord = False
    vertices = []
    obstacles_coord = [[]]
    earth_vertices = [[]]

    if manual_input == "n" or manual_input == "no":
        polygon_file = input_directory + "polygon.csv"
        config_file = input_directory + "config.ini"
        obstacles_directory = input_directory + "obstacles"

        if not os.path.exists(polygon_file):
            print("File missing : " + polygon_file)
            exit(1)
        if not os.path.exists(config_file):
            print("File missing : " + config_file)
            exit(1)
        if not os.path.exists(polygon_file):
            print("File missing : " + obstacles_directory)
            exit(1)

        # Read the different files to extract the required parameters
        earth_coord = True
        earth_vertices = read_vertices(polygon_file)
        earth_obstacles = read_many_vertices(obstacles_directory)

        vertices = convert_vertices(earth_vertices)
        if earth_obstacles:
            obstacles_coord = [convert_vertices_with_ref(obstacle, earth_vertices[0]) for obstacle in earth_obstacles]

        type_of_optimisation, radius = read_config(config_file)
        if problem == 1:
            if type_of_optimisation == 0 or type_of_optimisation == 1 or type_of_optimisation == 2:
                method = "cpp"
            elif type_of_optimisation == "concorde":
                method = "concorde"
            elif type_of_optimisation == "fast_tsp":
                method = "fast_tsp"
            elif type_of_optimisation == "tsp_solver2":
                method = "tsp_solver2"
            elif type_of_optimisation == "solve_tsp_simulated_annealing":
                method = "solve_tsp_simulated_annealing"
            elif type_of_optimisation == "solve_tsp_local_search":
                method = "solve_tsp_local_search"
        if problem == 0:
            if type_of_optimisation == 0 or type_of_optimisation == 1 or type_of_optimisation == 2:
                method = "python"
            elif type_of_optimisation == "tsp_to_dubins":
                method = "tsp_to_dubins"

    elif manual_input == "y" or manual_input == "yes":
        input_error = 0
        fixed_wings = False
        if problem == 0:
            fixed_wings = True

        if problem == -5:
            input_error = 1
            print("If the configuration is manual you have to select the type of problem to solve"
                  " (Fixed-wings UAV or multi-copter)\n 0 = Fixed-wings UAV / 1 =  multi-copter")
        if choice_of_vertex == -5:
            input_error = 1
            print("If the configuration is manual you have to select an area to cover (value between 0 and 5)")
        if choice_of_obstacles == -5:
            input_error = 1
            print("If the configuration is manual you have to select the obstacles configuration (-1 no obstacles and "
                  "0 some obstacles)")
        if type_of_optimisation == -5:
            input_error = 1
            print("If the configuration is manual you have to select the algorithm you want to use (values "
                  "depend of the problem you want to solve):\n"
                  "Problem = 1: 0 = SA / 1 = GA / 2 = SLS / 3 = concorde/4 = fast_tsp / 5 = tsp_solver2 / "
                  "6 = solve_tsp_simulated_annealing / 7 = solve_tsp_local_search\n "
                  "Problem = 0 : 0 = SA / 1 = GA / 2 = SLS / 3 = tsp_to_dubins")
        if radius == -5:
            input_error = 1
            print("If the configuration is manual you have to select a coverage radius (value depend of the area you "
                  "choose)")
        if turning_radius == -5 and fixed_wings:
            input_error = 1
            print("If the configuration is manual you have to select a turning radius for your fixed-wings UAV")

        if not 0 <= problem <= 1:
            input_error = 1
            print("Problem must be either 0 or 1 (Fixed-wings UAV or multi-copter)\n"
                  " 0 = Fixed-wings UAV / 1 =  multi-copter")
        if not 0 <= choice_of_vertex <= 5:
            input_error = 1
            print("polygon: between 0 and 5 ")
        if choice_of_obstacles != -1 and choice_of_obstacles != 0:
            input_error = 1
            print("obstacle : -1 for no obstacle, 0 for some obstacles")
        if not 0 <= type_of_optimisation <= 7 and problem == 1:
            input_error = 1
            print("optimisation algorithm:\n"
                  "Problem = 1: between 0 and 7  (0 = SA / 1 = GA / 2 = SLS / 3 = concorde / 4 = "
                  "fast_tsp / 5 = tsp_solver2 / 6 = solve_tsp_simulated_annealing/ 7 = solve_tsp_local_search)")
        if not 0 <= type_of_optimisation <= 3 and problem == 1:
            input_error = 1
            print("optimisation algorithm:\n"
                  "Problem = 0 : between 0 and 3 ((0 = SA / 1 = GA / 2 = SLS / 3 = tsp_to_dubins)")
            # TODO : don't forget to add new methods
        if radius <= 0:
            input_error = 1
            print("Radius must be a strictly positive integer ( value depend of your polygon)")
        if turning_radius <= 0 and fixed_wings:
            input_error = 1
            print("Turning radius must be a strictly positive integer (value depend of your polygon)")

        if input_error == 1:
            print("Usage example : "
                  "\n multi-copter: python3 main.py --manual y --problem 1 --polygon 0 --obstacles -1 --algo 3 "
                  "--radius 10 "
                  "\n fixed-wings: python3 main.py --manual y --problem 0 --turning_radius 10 --polygon 0 --obstacles "
                  "-1 --algo 3 --radius 10")
            exit(1)

        if problem == 1:
            if type_of_optimisation == 3:
                method = "concorde"
            elif type_of_optimisation == 4:
                method = "fast_tsp"
            elif type_of_optimisation == 5:
                method = "tsp_solver2"
            elif type_of_optimisation == 6:
                method = "solve_tsp_simulated_annealing"
            elif type_of_optimisation == 7:
                method = "solve_tsp_local_search"
        if problem == 0:
            if type_of_optimisation == 3:
                method = "tsp_to_dubins"

        # This parameter is used to let the program know which kind of coordinates it receive
        # True only if geodetic coordinates
        earth_coord = False
        # Different Polygons
        if choice_of_vertex == 0:
            vertices = [[483.5, 357.5], [4., 308.], [39., 5.], [529., 57.]]
            # Typical radius value : 12

        elif choice_of_vertex == 1:
            vertices = [[117.5, 282], [7.5, 152.5], [120, 100], [92.5, 8.], [280.5, 13.], [314., 133.], [174., 126.],
                        [219., 234.], [132., 229.]]
            # Typical radius value : 8

        elif choice_of_vertex == 2:
            vertices = [[0, -100], [1000, -100], [1000, 100], [450, 100], [450, 400], [2500, 600], [2600, 2400],
                        [-500, 2000], [0, 550], [250., 350], [250, 100], [0, 100]]
            # Typical radius value : 90

        elif choice_of_vertex == 3:
            earth_coord = True
            # List of Earth coordinates (lat,lon)
            earth_vertices = [[55.588681, 10.142625], [55.604044, 10.006676], [55.696094, 9.998907],
                              [55.722355, 10.643693], [56.014398, 10.993276], [56.079480, 11.257405],
                              [56.219006, 11.366479], [56.258795, 11.542008], [56.032347, 11.800682],
                              [55.985864, 11.627463], [56.024604, 11.324906], [55.925093, 11.066232],
                              [55.630238, 10.648196], [55.636757, 10.253256], [55.600239, 10.213993]]

            vertices = convert_vertices(earth_vertices)
            # Typical radius value : 1500

        elif choice_of_vertex == 4:
            earth_coord = True
            radius_of_circle = 3000
            earth_vertices = [[51.2150, 2.5311]]

            vertices = create_pseudo_circle([0, 0], radius_of_circle, 50)
            # Typical radius value : 220

        elif choice_of_vertex == 5:
            vertices = [[-0.1, -0.1], [100.1, -0.1], [100.1, 100.1], [-0.1, 100.1]]
            # Typical radius : 8

        # Obstacles
        if choice_of_obstacles == 0:
            obstacles_coord = [[[175, 40], [175, 100], [250, 100], [250, 40]],
                               [[100, 150], [100, 200], [150, 200], [150, 150]], [[120, 40], [120, 60], [140, 60]]]
        elif choice_of_obstacles == -1:
            obstacles_coord = [[]]

    else:
        print("Only accept y,yes or n,no as answers")
        exit(1)

    # Transform list into polygons
    polygon = Polygon(vertices)
    penalty_on_obstacles = polygon.length * 10
    min_x, min_y, max_x, max_y = polygon.bounds

    polygone_obstacles = []
    for obstacle_coord in obstacles_coord:
        polygone_obstacles.append(Polygon(obstacle_coord))

    # ------------------ Begin initialization ---------------------------

    print("Initialization of points...")
    diameter = radius * 2
    if point_generation == "random":
        points = generate_points_inside_polygon(polygon, polygone_obstacles, int(diameter))
    elif point_generation == "systematic":
        points = create_grid(polygon, polygone_obstacles, int(diameter * 0.85))
    else:
        points = [[]]
    nb_points = len(points)
    print("There are ", nb_points, "points in the path")

    print("Compute matrix of distance...")
    dist_matrix = create_dist_matrix(nb_points, points, vertices, obstacles_coord, penalty_on_obstacles)

    # initialize a starting solution for iterative improvement algorithm
    path = [i for i in range(nb_points)]
    final_sol = []

    # --------------- Start the resolution of the problem -----------------

    print("Find optimized solution")
    # Record the start time
    optimisation_start_time = time.time()
    if problem == 1:
        if method == "cpp":
            final_sol = cpp_opti(path, dist_matrix, type_of_optimisation, cpp_executable_path)
            final_sol = cpp_opti(final_sol, dist_matrix, 2, cpp_executable_path)
        elif method == "python":
            final_sol = optimise(path, dist_matrix, type_of_optimisation)
        elif method == "concorde":
            final_sol = solve_concorde(dist_matrix)
        elif method == "fast_tsp":
            final_sol = fast_tsp.find_tour(dist_matrix)  # 2 seconds as defaults time limit value
        elif method == "tsp_solver2":
            final_sol = solve_tsp(dist_matrix)
        elif method == "solve_tsp_simulated_annealing":
            final_sol, distance = solve_tsp_simulated_annealing(dist_matrix)
        elif method == "solve_tsp_local_search":
            final_sol, distance = solve_tsp_local_search(dist_matrix)

    if problem == 0:
        if method == "python" or method == "cpp":
            final_sol = optimise_dubins(path, points, turning_radius, type_of_optimisation)
        if method == "tsp_to_dubins":
            final_sol = fast_tsp.find_tour(dist_matrix)
            length, _ = compute_dubins_paths_and_length(points, final_sol, turning_radius)
            print(length)
            final_sol = optimise_dubins(final_sol, points, turning_radius, type_of_optimisation=2)

    # Record the end time
    optimisation_end_time = time.time()
    # Calculate the elapsed time
    optimisation_elapsed_time = optimisation_end_time - optimisation_start_time
    print(f"Elapsed time: {optimisation_elapsed_time:.4f} seconds")

    # --------------- Output Processing -----------------------------------

    # Represent final solution as a list of points
    true_sol = []
    for i in final_sol:
        true_sol.append(list(points[i]))
    polygon2 = Polygon(true_sol)
    if problem == 1:
        print("Real distance : ", compute_true_distance(true_sol))
    if problem == 0:
        pass

    # Transform relative coordinates back to geographical one if needed
    real_path = []
    if earth_coord:
        for i in true_sol:
            real_path.append(convert_relative_to_coordinates(earth_vertices[0][0], earth_vertices[0][1], i))
        print("Path for the UAV in geographical coordinates : \n", real_path)

    # Record the end time
    global_end_time = time.time()
    # Calculate the elapsed time
    global_elapsed_time = global_end_time - global_start_time

    # Format the timestamp as per your requirements
    formatted_time = start_timestamp
    folder_path = save_directory + formatted_time

    # ----------Write outputs in results directory ---------------------------------

    # Copy the initial configuration path
    if manual_input == "y" or manual_input == "yes":
        os.makedirs(folder_path, exist_ok=True)
    else:
        shutil.copytree(input_directory, os.path.join(folder_path, 'initial_config'))

    if problem == 1:
        # Visualization using Matplotlib and save the image in the save_directory for multi-copter problem
        with PdfPages(folder_path + "/" + "Coverage.pdf") as pdf:
            plt.xlim(min_x, max_x)
            plt.ylim(min_y, max_y)
            # Plot the zone that is covered
            data_linewidth_plot(*polygon2.exterior.xy, color='green', linewidth=diameter, alpha=0.5)
            plt.plot(*polygon2.exterior.xy, color='green', linewidth=1)
            # Plot the polygone
            plt.plot(*polygon.exterior.xy, color='blue', linewidth=2)
            # Plot the obstacles
            for polygone_obstacle in polygone_obstacles:
                plt.plot(*polygone_obstacle.exterior.xy, color='pink', linewidth=1)
            # Plot the different points of the path
            plt.scatter(points[:, 0], points[:, 1], color='red', s=1)
            plt.axis('equal')
            pdf.savefig()
            plt.clf()

    if problem == 0:
        # Visualization using Matplotlib and save the image in the save_directory for fixed-wings problem
        with PdfPages(folder_path + "/" + "Dubins_Coverage.pdf") as pdf:

            # Compute Dubins paths and total length
            Dubins_length, dubins_paths = compute_dubins_paths_and_length(points, final_sol, turning_radius)

            print("Real Dubins distance : ", Dubins_length)

            x_dubins = []
            y_dubins = []
            for point in dubins_paths:
                xs, ys, _ = point
                x_dubins.append(xs)
                y_dubins.append(ys)

            plt.xlim(min_x, max_x)
            plt.ylim(min_y, max_y)
            # Plot the zone that is covered
            data_linewidth_plot(x_dubins, y_dubins, color='green', linewidth=diameter, alpha=0.5)
            plt.plot(x_dubins, y_dubins, color='green', linewidth=1)
            # Plot the polygone
            plt.plot(*polygon.exterior.xy, color='blue', linewidth=2)
            # Plot the obstacles
            for polygone_obstacle in polygone_obstacles:
                plt.plot(*polygone_obstacle.exterior.xy, color='pink', linewidth=1)
            # Plot the different points of the path
            plt.scatter(points[:, 0], points[:, 1], color='red', s=1)
            plt.axis('equal')
            pdf.savefig()
            plt.clf()

    # Write all the outputs in a timestamp file
    with open(folder_path + "/" + "output.txt", 'w') as output_file:
        print("Polygone : ", choice_of_vertex, file=output_file)
        print("Coverage radius : ", radius, file=output_file)
        print("Optimisation algorithm : ", type_of_optimisation, file=output_file)
        print("Method of optimisation : ", method, file=output_file)
        print("Obstacles : ", choice_of_obstacles, file=output_file)
        print("Start time : ", start_timestamp, file=output_file)
        print("Total computation time : ", global_elapsed_time, file=output_file)
        print("Optimisation computation time : ", optimisation_elapsed_time, file=output_file)
        if problem == 0:
            print("Turning_radius : ", turning_radius, file=output_file)
            print("Real Dubins distance : ", Dubins_length, file=output_file)
        if problem == 1:
            print("Real distance : ", compute_true_distance(true_sol), file=output_file)
        print("Total number of point to cover : ", nb_points, file=output_file)

        print("Path for the UAV in geographical coordinates : \n", list(real_path), file=output_file) \
            if earth_coord else print("Path for the UAV in relative coordinates : \n", list(true_sol), file=output_file)
        print("------------------------------------------------------------------------------", file=output_file)
        output_file.close()

    with open(folder_path + "/" + "optimisation_log.dat", 'w') as output_file:
        print("Start time : ", start_timestamp, file=output_file)
        print("Total computation time : ", global_elapsed_time, file=output_file)
        print("Optimisation computation time : ", optimisation_elapsed_time, file=output_file)
        if problem == 0:
            print("Real Dubins distance : ", Dubins_length, file=output_file)
        if problem == 1:
            print("Real distance : ", compute_true_distance(true_sol), file=output_file)
        print("Total number of point to cover : ", nb_points, file=output_file)
        print("------------------------------------------------------------------------------", file=output_file)
        output_file.close()

    with open(save_directory + "All_output_log.dat", 'a') as output_file:
        print("Polygone : ", choice_of_vertex, file=output_file)
        print("Coverage radius : ", radius, file=output_file)
        print("Optimisation algorithm : ", type_of_optimisation, file=output_file)
        print("Method of optimisation : ", method, file=output_file)
        print("Obstacles : ", choice_of_obstacles, file=output_file)
        print("Start time : ", start_timestamp, file=output_file)
        print("Total computation time : ", global_elapsed_time, file=output_file)
        print("Optimisation computation time : ", optimisation_elapsed_time, file=output_file)
        if problem == 0:
            print("Turning_radius : ", turning_radius, file=output_file)
            print("Real Dubins distance : ", Dubins_length, file=output_file)
        if problem == 1:
            print("Real distance : ", compute_true_distance(true_sol), file=output_file)
        print("Total number of point to cover : ", nb_points, file=output_file)
        print("------------------------------------------------------------------------------", file=output_file)
        output_file.close()

# -----------------------LICENSE--------------
"""
Copyright (c) 2022 Søren Mulvad. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

You are under no obligation whatsoever to provide any bug fixes, patches, or
upgrades to the features, functionality or performance of the source code
("Enhancements") to anyone; however, if you choose to make your Enhancements
available either publicly, or directly to the author of this software, without
imposing a separate written license agreement for such Enhancements, then you
hereby grant the following license: a non-exclusive, royalty-free perpetual
license to install, use, modify, prepare derivative works, incorporate into
other computer software, distribute, and sublicense such enhancements or
derivative works thereof, in binary and source code form.
"""

"""
Copyright 2017-2018 Joris Vankerschaver

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
following conditions are met: 

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following 
disclaimer. 

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
following disclaimer in the documentation and/or other materials provided with the distribution. 

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote 
products derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
MIT License

Copyright (c) 2020 Fillipe Goulart

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
