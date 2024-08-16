# Coverage of area using UAV

## Introduction
Unmanned Aerial Vehicles (UAVs), or drones, have revolutionized various industries with their ability to efficiently
navigate diverse terrains and access remote locations. Their precision and adaptability make them essential for 
such as precision agriculture, infrastructure monitoring, and search and rescue operations.
As the need for effective coverage of expansive areas with varying terrain grows, fixed-wing UAVs are increasingly
preferred for their autonomy and efficiency in large-scale mapping and surveillance.
However, their limited rotation capability poses challenges in navigating tight spaces and avoiding obstacles.

Therefor we implemented 4 different method to solve the problem : Boustrophedon Method, Iterated Dubins Simulation Method,
Heuristic for the Dubins Traveling Sells-man Problem (DTSP) and TSP Relaxation Methods.
These methods aim to improve UAV mission capabilities by addressing the challenges of path planning in complex environments,
enhancing the efficiency and effectiveness of autonomous aerial operations.

This project comprises a Python application that serves as the main user interface and a C++ program that
implements optimisation algorithms to achieve more efficient computations.
The project use 

The implemented solution demonstrates the effectiveness of strategic waypoint placement and optimisation techniques in
minimizing travel distances during area coverage. By combining the versatility of Python with the efficiency of C++,
this project offers a comprehensive solution for efficient UAV-based coverage.

The developed algorithm has the ability to work with areas which are delimited by polygons where the vertices are
expressed both in terms of local North-East (NE) and geodetic coordinates.

## Methodology
The method used in this project is as follows:
1. Analyse the input area as a polygon and transform the geodetic coordinates if necessary and do the same with the obstacles.
2. Place point in the polygons but outside the obstacles.
3. Compute a distance matrix with the distances between all the different points and add penalties when line linking two points
   encounter an edge of the polygon or an obstacle.
4. Launch one of the different optimisation algorithms, to find the best possible path.
5. Plot the solution and write it in the "results" directory.

## Algorithms
### Boustrophedon Method:
The boustrophedon approach is used to solve path planning problems by covering polygonal areas while avoiding obstacles.
It involves dividing the area into convex cells and generating a path that systematically covers these cells.
The final path is optimized using Dubins paths to accommodate the UAV's turning radius,
ensuring efficient coverage while navigating through obstacles.

### Dubins Simulation Method:
This method builds an iterative solution that is refined using a local search algorithm.
Initially, the area is split into convex cells, and a Dubins vehicle simulation is performed to generate
paths for each cell. These paths are then combined and optimized using a simple local search technique,
resulting in a globally optimized trajectory that respects the UAV's turning constraints.

### Heuristic Methods:
Heuristic methods are essential for tackling optimization problems where exact solutions are impractical.
These techniques leverage domain-specific knowledge and approximations to efficiently explore large search spaces
and identify near-optimal solutions. This research employs three key heuristic approaches: Genetic Algorithms (GA),
Simple Local Search (SLS), and Simulated Annealing (SA). Each method is tailored to address specific challenges in
path planning for fixed-wing UAVs, incorporating Dubins cost evaluations to adapt to the UAV's movement constraints
and enhance overall performance.

### TSP Relaxation Method:
The TSP Relaxation Method focuses on adapting traditional TSP solutions to the specific requirements of fixed-wing
UAV operations. By starting with a TSP solution from a standard solver, this approach modifies the solution to
accommodate the constraints of fixed-wing flight paths, such as turning radius and maneuverability limits.
This relaxation strategy aims to provide a practical, sub-optimal solution that aligns with the UAV's operational
constraints, making it a useful alternative to more rigid optimization techniques.

## Report

A report of the project containing a benchmark and analysis of the different algorithms can be found in the "Master_Thesis_Fixed_Wings_UAV_Mavros_Mickail.pdf" document.

## Folder Structure

This root folder content is defined in the following list.
1. The file **README.md** which is this file with the description, and setup and user instructions to use this software.
2. The report **UAV_report.pdf** with the analysis and comparison between the different algorithms performances.
3. The folder **cpp_optimisation** contains the C++ program that enables a faster optimisation process
4. The folder **python_code** contains the main Python algorithm.
5. The folder **results** includes different folder containing all information regarding output and configuration of the
   program (see output section), it also includes the different instances solved for the benchmarking in the sub folder **TestResults**.
6. The folder **configuration** which contains "polygon.csv", "obstacles directory", "config.ini" used to automatically
   launch the program (see usage section)
7. The folder **resource** which contains : the file *requirements.txt* with the list of all the Python project's dependencies
   and the *concorde.py* file that should be used in the py_concorde library
   ( see requirement).


## Requirements
The following dependencies are required for the project to run:
- Python >= 3.8.10
- g++ >= 9.4.0

The *py_concorde* library that you need to install following these line (more info in the [pyconcorde documentation](https://github.com/jvkersch/pyconcorde)).

The following modules are imported in the Python code:
- matplotlib
- shapely
- pyproj
- numpy

For image-based area coverage the following additional modules are used:
- pillow
- scikit-learn
- cv-2

This project was developed and compatibility was tested on the WSL Ubuntu 20.04 distribution.

## Installation

Follow the next steps to set up this project:

1. To install the required Python modules, which are listed in the *requirement.txt* file, run the following command on the console:
   ```
   pip install -r resource/requirement.txt
   ```
2. To install the py_concord library (see more info in this [link](https://github.com/jvkersch/pyconcorde)),
   run the following command in any of your directories (not especially this project).
   ```
   git clone https://github.com/jvkersch/pyconcorde
   cd pyconcorde
   pip install -e .
   ```
   If the project is not running when using the "concorde" method due to an error with the subprocess, you can try replacing the *concorde.py* file in
   your *pyconcorde/concorde* directory with the one present in the *resource* folder.
3. To install the "pydubins" library (see more info in this [link](https://github.com/AndrewWalker/pydubins/tree/master)),
   run the following command in any of your directories (not especially this project).
   ```
   git clone https://github.com/AndrewWalker/pydubins
   pip install .
   ```
   Error can occur during the installation due to lack of compatibility with python version >=3.9, you can therefore
   follow the guidelines in the following [link](https://github.com/AndrewWalker/pydubins/issues/16#issuecomment-1779758323).
4. Given that the project contain a c++ program you first need to compile it before using the python code,
   therefore clean up the compiled files (if needed):
    ```
    make clean -C ./cpp_optimisation
    ```
5. Next, compile the C++ program:
    ```
    make -C ./cpp_optimisation
    ```
## Usage

1. Launch the Python application:
    ```
    cd python_code/
    python3 ./main.py
    ```

2. There are two options to launch the program : one manual and the other automatic

   a . **Manual** : Enter your inputs for the instance you want the algorithm to resolve
    - The polygon you want to cover, value between 0 and 5
    - The obstacles you want to use, 0 for some obstacles, -1 without obstacles
    - Problem nature of the problem to solve :
      - Problem = 1 : for classical TSP resolution
      - Problem = 0 : for resolution for fixed wings UAV
    - The optimisation algorithm you want to use, 
      - If problem = 1 : 0 = SA / 1 = GA / 2 = SLS / 3 = concorde /
        4 = fast_tsp / 5 = tsp_solver2 / 6 = solve_tsp_simulated_annealing / 7 = solve_tsp_local_search
      - If problem = 0 : 0 = SA / 1 = GA / 2 = SLS / 3 = tsp_to_dubins / 4 = boustrophedon / 5 = simulation
    - The radius of the area covered by your UAV.

   Example :
   - Multicopter:
   ```
   python3 main.py --manual y --problem 1 --polygon 0 --obstacles -1 --algo 3 --radius 10
   ```
   - Fixed-Wings:
   ```
   python3 main.py --manual y --problem 0 --turning_radius 10 --polygon 0 --obstacles -1 --algo 3 --radius 10
   ```
b. **Automatic**: The program automatically read the files inside the configuration directory.
- *polygon.csv* : containing a representation of the polygon in geodetic coordinates, using the format : lat, lon.
- *obstacles directory* : directory containing many csv files representing the different obstacles, using geodetic
  coordinates in format : lat, lon.
  *config.ini* : file containing the information for the program : algorithm to be used and
  the coverage radius of the UAV.

Example:
  ```
  python3 main.py --manual n
  ```

## Benchmark

It is also possible to run many instances of the project one after another using the **benchmarking.py** file.
To run it you can use the following commands:
```
cd python_code/
python3 ./benchmarking.py
```


## Output
All outputs can be found in the "results" folder. The different outputs are written in a directory named with the date
and time of the software execution start (format :YYYYMMDDHHMMSSZZZZZ). This directory contains:
- Folder with the initial configuration if the process was called using the automatic way.
- The path, polygon, obstacles and the effectively covered area are plotted in a file called "Coverage.pdf".
- The final path and its length are plotted in the terminal and in the "output.txt" file.
- The optimisation log with the start time, Total computation time, optimisation computation time and real distance.


## Image Processing

The ImageProcess.py module offers functions to cover areas indicated on images.
This involves utilizing libraries like pillow, cv2, and scikit-learn for image
processing and analysis.




