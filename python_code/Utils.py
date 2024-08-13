import configparser
import os
import csv
from math import sqrt
import pyproj
import numpy as np
from numpy import full
from matplotlib import pyplot as plt

from shapely.geometry import Point, Polygon, MultiPolygon, GeometryCollection, box
from shapely.ops import unary_union

# Create a Pyproj transformer for geodetic operations
transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)


class data_linewidth_plot:
    """
    Class that allow to draw line with a size adapted to the plot
    """

    def __init__(self, x, y, **kwargs):
        self.ax = kwargs.pop("ax", plt.gca())
        self.fig = self.ax.get_figure()
        self.lw_data = kwargs.pop("linewidth", 1)
        self.lw = 1
        self.fig.canvas.draw()

        self.ppd = 72. / self.fig.dpi
        self.trans = self.ax.transData.transform
        self.linehandle, = self.ax.plot([], [], **kwargs)
        if "label" in kwargs: kwargs.pop("label")
        self.line, = self.ax.plot(x, y, **kwargs)
        self.line.set_color(self.linehandle.get_color())
        self._resize()
        self.cid = self.fig.canvas.mpl_connect('draw_event', self._resize)

    def _resize(self, event=None):
        lw = ((self.trans((1, self.lw_data)) - self.trans((0, 0))) * self.ppd)[1]
        if lw != self.lw:
            self.line.set_linewidth(lw)
            self.lw = lw
            self._redraw_later()

    def _redraw_later(self):
        self.timer = self.fig.canvas.new_timer(interval=10)
        self.timer.single_shot = True
        self.timer.add_callback(lambda: self.fig.canvas.draw_idle())
        self.timer.start()


def create_pseudo_circle(center, radius, num_sides):
    """
    Formula to approximate a circle as a polygon
    :param center:
    :param radius:
    :param num_sides: number of side for the approximation
    :return: the vertices of the polygon representing the circle
    """
    vertices = []
    for i in range(num_sides):
        angle = 2 * np.pi * i / num_sides
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        vertices.append((x, y))
    return vertices


def read_vertices(csv_file):
    """
    Read a csv file and extract the vertices
    :param csv_file: csv file containing the latitude and longitude of an area (polygon)
    :return: the different vertices in a list
    """
    vertices = []
    with open(csv_file, newline='') as csvfile:
        csv_reader = csv.reader(csvfile)
        # Skip the header row if it exists
        next(csv_reader, None)
        # Iterate through each row in the CSV and append it to the list as floats
        for row in csv_reader:
            try:
                lat, lon = float(row[0]), float(row[1])
            except ValueError:
                print("Lat and Long value must be float value in : ", csv_file)
                exit(1)
            if -90 < lat < 90 and -180 < lon < 180:
                vertices.append([lat, lon])
            else:
                print("Lat and Long value in " + f'csvfile' + "must be : -90 < lat < 90 and -180 <"
                                                              " lon < 180 in : ", csv_file)
                exit(1)

    return vertices


def read_many_vertices(directory):
    """
    Read a directory containing many files representing polygons to extract the different vertices
    :param directory: the directory containing many polygons files
    :return: list of lists of vertices (polygons)
    """
    list_vertices = []

    for csv_file_name in os.listdir(directory):
        csv_file = os.path.join(directory, csv_file_name)

        if os.path.isfile(csv_file):
            vertices = read_vertices(csv_file)
            list_vertices.append(vertices)

    return list_vertices


def read_config(config_file):
    # Create a configparser object
    config = configparser.ConfigParser()
    # Read the configuration file
    config.read(config_file)

    # Retrieve values from the configuration file
    optimization_algorithm = config.get('UAVConfiguration', 'optimization_algorithm')

    try:
        coverage_radius = config.getint('UAVConfiguration', 'coverage_radius')
    except ValueError:
        print("Coverage_radius must be a number in : ", config_file)
        exit(1)

    if coverage_radius <= 0:
        print(" covergage_radius must be a positive integer in : ", config_file)
        exit(1)

    if optimization_algorithm == "simulated annealing":
        optimization_algorithm = 0
    elif optimization_algorithm == "genetic":
        optimization_algorithm = 1
    elif optimization_algorithm == "simple local search":
        optimization_algorithm = 2
    elif optimization_algorithm == "concorde" or optimization_algorithm == "fast_tsp" \
            or optimization_algorithm == "tsp_solver2" or optimization_algorithm == "solve_tsp_simulated_annealing" \
            or optimization_algorithm == "solve_tsp_local_search":
        pass
    else:
        print("Wrong algorithm in config file, choice between : simulated annealing, genetic, simple local "
              "search, concorde, fast_tsp, tsp_solver2, solve_tsp_simulated_annealing, solve_tsp_local_search "
              "in : ", config_file)
        exit(1)

    return optimization_algorithm, coverage_radius


def convert_relative_to_coordinates(ref_lat, ref_lon, relative_coordinates):
    """
    Convert given relative coordinate into geographical one using pyproj library
    :param ref_lat: latitude of the reference point
    :param ref_lon: longitude of the reference point
    :param relative_coordinates:
    :return: geographical coordinate of a point
    """
    x_change, y_change = relative_coordinates

    # Convert reference point's latitude and longitude to projected coordinates
    ref_x, ref_y = transformer.transform(ref_lon, ref_lat)  # Traditional GIS order

    # Calculate the new projected coordinates
    new_x = ref_x + x_change
    new_y = ref_y + y_change

    # Convert projected coordinates back to latitude and longitude
    new_lon, new_lat = transformer.transform(new_x, new_y, direction="INVERSE")  # Traditional GIS order

    return new_lat, new_lon


def convert_vertices(vertices):
    """
    Convert all the vertices from geographical coordinates into relative ones
    :param vertices: List of geographical coordinate
    :return: the vertices using relative coordinates
    """
    # Convert the first vertex to projected coordinates
    ref_x, ref_y = transformer.transform(vertices[0][1], vertices[0][0])

    # Convert each vertex to projected coordinates and make them relative to the reference
    converted_vertices = [
        ((transformer.transform(lon, lat)[0] - ref_x), (transformer.transform(lon, lat)[1] - ref_y))
        for lat, lon in vertices
    ]

    return converted_vertices


def convert_vertices_with_ref(vertices, ref_point):
    """
    Convert all the vertices from geographical coordinates into relative ones
    :param vertices: List of geographical coordinate
    :param ref_point: Point of reference for the transform
    :return: the vertices using relative coordinates
    """
    ref_x, ref_y = transformer.transform(ref_point[1], ref_point[0])

    # Convert each vertex to projected coordinates and make them relative to the reference
    converted_vertices = [
        ((transformer.transform(lon, lat)[0] - ref_x), (transformer.transform(lon, lat)[1] - ref_y))
        for lat, lon in vertices
    ]

    return converted_vertices


def intersect(polygon, p1, p2):
    """
    Function to check if a line linking to point intersect with a given polygone
    :param polygon:
    :param p1: point 1
    :param p2: point 2
    :return: Boolean
    """

    def orientation(p, q, r):
        """
        Function that check the orientation of 3 given points p,q,r
        The function calculates the cross-product of vectors (q-p) and (r-q) to determine the orientation
        """
        # (qy-py) * (rx-qx) - (qx-px)*(ry-qy)
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # Collinear
        return 1 if val > 0 else -1  # Clockwise or Counterclockwise

    def on_segment(p, q, r):
        """
        This function receive 3 different points.
        It determines whether q lies on a line segment defined by points p and r.
        It checks if q lies within the bounding box of the line segment formed by p and r.
        If q falls within this bounding box, it suggests that q is on the line segment.
        """
        return (max(p[0], r[0]) >= q[0] >= min(p[0], r[0]) and
                max(p[1], r[1]) >= q[1] >= min(p[1], r[1]))

    def do_intersect(edge1, edge2, point1, point2):
        o1 = orientation(edge1, edge2, point1)
        o2 = orientation(edge1, edge2, point2)
        o3 = orientation(point1, point2, edge1)
        o4 = orientation(point1, point2, edge2)
        # General case of intersection
        if o1 != o2 and o3 != o4:
            return True
        # Special cases when the points are collinear
        if o1 == 0 and on_segment(edge1, point1, edge2):
            return True
        if o2 == 0 and on_segment(edge1, point2, edge2):
            return True
        if o3 == 0 and on_segment(point1, edge1, point2):
            return True
        if o4 == 0 and on_segment(point1, edge2, point2):
            return True
        return False

    for i in range(len(polygon)):
        edge = (polygon[i], polygon[(i + 1) % len(polygon)])
        if do_intersect(edge[0], edge[1], p1, p2):
            return True

    return False


def initialize_points(radius, point_generation, polygon, polygone_obstacles):
    """
    Initialize different points inside a specified area
    :param radius: vision radius that influence the distance between the points
    :param point_generation: type of generation
    :param polygon: area to cover
    :param polygone_obstacles: obstacle to not cover
    :return:
    """
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
    return points, nb_points


def initialize_solution(nb_points):
    """
    Initialize a sequence of point
    :param nb_points: size of the sequence
    :return:
    """
    path = [i for i in range(nb_points)]
    final_sol = []
    return path, final_sol


def create_dist_matrix(nb, points, polygone, obstacles, penalty_on_obstacles):
    """
    Create a matrix of distance between all the point and add penalties if the line between two point cross an obstacle
    or edge of the polygone
    :param nb: number of points
    :param points: all the points
    :param polygone :(Polygon)
    :param obstacles :(array of Polygon) list of obstacles
    :param penalty_on_obstacles: chosen penalty
    :return: Matrix of distance between the points
    """
    matrix = full((nb, nb), 0)
    for i in range(nb):
        if i % 100 == 0: print("Number of row analysed:", i)
        for j in range(i, nb):
            if i == j:
                matrix[i][j] = 0
            elif not intersect(polygone, points[i], points[j]):
                intersection = False
                distance = sqrt((points[j][0] - points[i][0]) ** 2 + (points[j][1] - points[i][1]) ** 2)
                for obstacle in obstacles:
                    if intersect(obstacle, points[i], points[j]):
                        intersection = True
                        matrix[i][j] += penalty_on_obstacles + distance
                        matrix[j][i] += penalty_on_obstacles + distance
                if not intersection:
                    matrix[i][j] = distance
                    matrix[j][i] = distance
            else:
                distance = sqrt((points[j][0] - points[i][0]) ** 2 + (points[j][1] - points[i][1]) ** 2)
                matrix[i][j] += penalty_on_obstacles + distance
                matrix[j][i] += penalty_on_obstacles + distance

    return matrix


def compute_true_distance(path):
    """
    Return the distance to go through all the point in the path
    :param path: List of point in relative coordinates
    :return: the total distance between all the points
    """
    size = len(path)
    distance = 0
    for i in range(size):
        j = (1 + i) % size
        distance += sqrt((path[j][0] - path[i][0]) ** 2 + (path[j][1] - path[i][1]) ** 2)
    return distance


def generate_points_inside_polygon(polygon, obstacles, min_dist, max_attempts=50):
    """
    Generate uniformly distributed points inside a polygon using Poisson Disk Sampling
    :param polygon: (Polygon) The polygon where points will be generated
    :param min_dist: Minimum distance between points
    :param max_attempts: Maximum number of attempts to generate a new point around an existing point
    :param obstacles: (array of Polygon) Place were points can't be generated
    :return: An array of shape (N, 2) representing the generated points
    """

    def is_valid_point(point, obstacles, existing_points):
        for existing_point in existing_points:
            if np.linalg.norm(np.array(point) - np.array(existing_point)) <= min_dist * 0.9:
                return False
        for obstacle in obstacles:
            if obstacle.contains(Point(point[0], point[1])):
                return False
        return polygon.contains(Point(point[0], point[1]))

    def get_random_point_around(point):
        r = min_dist + np.random.rand() * min_dist
        theta = np.random.rand() * 2 * np.pi
        return point[0] + r * np.cos(theta), point[1] + r * np.sin(theta)

    # Get the bounding box of the polygon
    min_x, min_y, max_x, max_y = polygon.bounds

    # Generate initial point and add it to the list of points
    def generate_first_point(minx, miny, maxy, maxx, obstacles):
        point = (minx + (maxx - minx) * np.random.rand(), miny + (maxy - miny) * np.random.rand())
        point_obstacle_free = True
        for obstacle in obstacles:
            if obstacle.contains(Point(point[0], point[1])):
                point_obstacle_free = False
        return point, point_obstacle_free

    initial_point, obstacle_free = generate_first_point(min_x, min_y, max_x, max_y, obstacles)

    while not (polygon.contains(Point(initial_point[0], initial_point[1])) and obstacle_free):
        initial_point, obstacles = generate_first_point(min_x, min_y, max_x, max_y, obstacles)

    points = [initial_point]

    active_points = [initial_point]

    while active_points:
        random_index = np.random.randint(len(active_points))
        current_point = active_points[random_index]
        found_new_point = False

        for _ in range(max_attempts):
            new_point = get_random_point_around(current_point)

            if is_valid_point(new_point, obstacles, points):
                active_points.append(new_point)
                points.append(new_point)
                found_new_point = True
                break

        if not found_new_point:
            active_points.pop(random_index)

    return np.array(points)


def create_grid(polygon, obstacles, resolution):
    """
    Place point in a polygone in a systematic ways while dodging the obstacles
    :param polygon: (Polygon)
    :param obstacles: (array of polygon)
    :param resolution: distance between each point
    :return: List of point inside the polygon
    """
    # Calculate bounding box
    minx, miny, maxx, maxy = polygon.bounds

    # Generate grid points
    grid_points = []
    for x in range(int(minx - 0.00001), int(maxx + 0.00001), resolution):
        for y in range(int(miny - 0.00001), int(maxy + 0.00001), resolution):
            point = Point(x, y)
            obstacle_free = True
            for obstacle in obstacles:
                if obstacle.contains(point):
                    obstacle_free = False
            if polygon.contains(point) and obstacle_free:
                grid_points.append([x, y])
    return np.array(grid_points)


def is_convex(polygon):
    """
    Check if a polygon is convex.
    :param polygon:
    :return boolean
    """
    points = list(polygon.exterior.coords[:-1])  # Exclude the repeated first/last point
    if len(points) < 4:
        return True  # Triangles are always convex

    n = len(points)
    sign = None
    for i in range(n):
        dx1 = points[(i + 1) % n][0] - points[i][0]
        dy1 = points[(i + 1) % n][1] - points[i][1]
        dx2 = points[(i + 2) % n][0] - points[(i + 1) % n][0]
        dy2 = points[(i + 2) % n][1] - points[(i + 1) % n][1]
        cross_product = dx1 * dy2 - dy1 * dx2
        current_sign = cross_product > 0
        if sign is None:
            sign = current_sign
        elif sign != current_sign:
            return False
    return True


def find_critical_points(polygon, obstacles):
    """
    Find the critical points for Boustrophedon decomposition.

    :param obstacles:
    :return:
    """
    critical_points = list(polygon.exterior.coords)
    for obstacle in obstacles:
        critical_points.extend(list(obstacle.exterior.coords))
    return critical_points


def round_polygon_coords(polygon, precision=1):
    return Polygon([(round(x, precision), round(y, precision)) for x, y in polygon.exterior.coords])


def boustrophedon_decomposition(polygon, obstacles, min_strip_height):
    """
    Perform Boustrophedon Cell Decomposition on the given polygon.
    :param polygon: The polygon to decompose.
    :param obstacles(List[Polygon]): List of obstacles to subtract.
    :param min_strip_height:  Minimum height of the decomposed cells.
    :return: List[Polygon]: List of decomposed cells.
    """

    critical_points = find_critical_points(polygon, obstacles)
    critical_points = [(round(x, 1), round(y, 1)) for x, y in critical_points]
    critical_points = sorted(set(critical_points), key=lambda p: p[1])  # Sort by y-coordinate and remove duplicates

    minx, miny, maxx, maxy = polygon.bounds
    minx, miny, maxx, maxy = round(minx, 1), round(miny, 1), round(maxx, 1), round(maxy, 1)
    cells = []

    for i in range(len(critical_points) - 1):
        y1 = critical_points[i][1]
        y2 = critical_points[i + 1][1]

        # Skip horizontal segments and strips smaller than min_strip_height
        if y1 == y2 or (y2 - y1) < min_strip_height:
            continue

        strip = box(minx, y1, maxx, y2)
        if strip.intersects(polygon):
            intersection = strip.intersection(polygon)
            if isinstance(intersection, (MultiPolygon, GeometryCollection)):
                for poly in intersection.geoms:
                    if isinstance(poly, Polygon):
                        cells.append(round_polygon_coords(poly))  # Round coordinates here
            elif isinstance(intersection, Polygon):
                cells.append(round_polygon_coords(intersection))  # Round coordinates here

    return cells


def subtract_obstacles(cells, obstacles):
    """
    Subtract obstacles from each cell.
    :param cells(List[Polygon]): List of cells to subtract obstacles from.
    :param obstacles(List[Polygon]): List of obstacles to subtract.
    :return: List[Polygon]: List of cells with obstacles subtracted.
    """
    result_cells = []
    for cell in cells:
        for obstacle in obstacles:
            cell = cell.difference(obstacle)
        if not cell.is_empty:
            if isinstance(cell, (MultiPolygon, GeometryCollection)):
                result_cells.extend([poly for poly in cell.geoms if isinstance(poly, Polygon)])
            elif isinstance(cell, Polygon):
                result_cells.append(cell)
    return result_cells


def merge_polygons(polygons):
    """
     Merge polygons if the result is convex. Iteratively try to merge every pair of polygons.
    :param polygons(list of Polygon): List of polygons to be merged.
    :return:  list of Polygon: The merged convex polygons and untouched original polygons.
    """
    if len(polygons) == 0:
        raise ValueError("No polygons to merge.")

    # Ensure all polygons are valid
    for poly in polygons:
        if not poly.is_valid:
            raise ValueError("All input polygons must be valid.")

    merged_polygons = []
    merged = [False] * len(polygons)  # Keep track of which polygons have been merged

    for i in range(len(polygons)):
        if merged[i]:
            continue
        for j in range(i + 1, len(polygons)):
            if merged[j]:
                continue
            # Attempt to merge polygons[i] and polygons[j]
            potential_merge = unary_union([polygons[i], polygons[j]])
            if potential_merge.geom_type == 'Polygon' and is_convex(potential_merge):
                merged_polygons.append(potential_merge)
                merged[i] = merged[j] = True
                break  # Once merged, break to avoid further checks for polygons[i]
        if not merged[i]:
            # If polygons[i] was not merged, add it as is
            merged_polygons.append(polygons[i])

    return merged_polygons


def boustrophedon_cells(polygon, obstacle_polygons, min_strip_height):
    """
    Generate convex cells in a polygon with obstacles inside
    :param polygon: area to cover
    :param obstacle_polygons:
    :param min_strip_height: minimum height of sub areas
    :return: list of polygons representing the new sub polygons
    """
    # Perform Boustrophedon Decomposition
    cells = boustrophedon_decomposition(polygon, obstacle_polygons, min_strip_height)
    # Subtract obstacles from the cells
    cells_with_obstacles = subtract_obstacles(cells, obstacle_polygons)
    cells_with_obstacles = merge_polygons(cells_with_obstacles)

    return cells_with_obstacles
