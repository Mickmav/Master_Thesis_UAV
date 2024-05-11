#
# # Example vertices (geographical coordinates)
# vertices = [[55.5886810, 10.142625], [55.604044, 10.006676], [55.696094, 9.998907],
#             [55.722355, 10.643693], [56.014398, 10.993276], [56.079480, 11.257405],
#             [56.219006, 11.366479], [56.258795, 11.542008], [56.032347, 11.800682],
#             [55.985864, 11.627463], [56.024604, 11.324906], [55.925093, 11.066232],
#             [55.630238, 10.648196], [55.636757, 10.253256], [55.600239, 10.213993]]
#
# converted_relative_coords = convert_vertices(vertices)
# print(converted_relative_coords)
#
# new_vert = []
# for i in converted_relative_coords:
#     new_vert.append(convert_relative_to_coordinates(vertices[0][0], vertices[0][1], i))
# print(vertices)
# print(new_vert)
#
# polygon = Polygon(converted_relative_coords)
# plt.plot(*polygon.exterior.xy, color='blue', linewidth=2)
#
# plt.axis('equal')
# plt.show()

from concorde.problem import Problem
from concorde.concorde import Concorde
import numpy as np


def symmetricize(m, high_int=None):
    # if high_int not provided, make it equal to 10 times the max value:
    if high_int is None:
        high_int = round(10 * m.max())

    m_bar = m.copy()
    np.fill_diagonal(m_bar, 0)
    u = np.matrix(np.ones(m.shape) * high_int)
    np.fill_diagonal(u, 0)
    m_symm_top = np.concatenate((u, np.transpose(m_bar)), axis=1)
    m_symm_bottom = np.concatenate((m_bar, u), axis=1)
    m_symm = np.concatenate((m_symm_top, m_symm_bottom), axis=0)

    return m_symm.astype(int)  # Concorde requires integer weights


def solve_concorde(matrix):
    print(matrix)
    problem = Problem.from_matrix(matrix)
    solver = Concorde()
    solution = solver.solve(problem)
    print(f'Optimal tour: {solution.tour}')
    return solution.tour



dist_matrix = np.matrix([[0, 99, 99, 0, 7, 3],
                         [99, 0, 99, 5, 0, 4],
                         [99, 99, 0, 2, 4, 0],
                         [0, 5, 2, 0, 99, 99],
                         [7, 0, 4, 99, 0, 99],
                         [3, 4, 0, 99, 99, 0]])

dist_matrix = symmetricize(dist_matrix)
final_sol = solve_concorde(dist_matrix)
print(final_sol)
