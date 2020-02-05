# Dependencies (numpy, scipy, matplotlib)
import numpy
from scipy.spatial import ConvexHull, Delaunay
from scipy.optimize import linprog
import matplotlib.pyplot as plt

# Global variables for center of mass
x_com = 0
y_com = 0
z_com = 0

# Global lists storing occupied coordinates for COM calculations
mass_list_x = []
mass_list_y = []
mass_list_z = []

# Global lists to create support polygon point cloud
support_list_x = []
support_list_z = []

# Standardized unit of mass
mass = 1

# Initializing example 3D input function would receive (params x,y,z)
array = numpy.zeros((6, 10, 6))

########################################################################
''' This block generates a build that should fail '''

# # Adding 1x1x2 block (length x width x height)
# array[1][0][0] = '1'
# array[1][1][0] = '1'
#
# # Adding 4x1x1 block on top of 1x1x2 block
# array[1][2][0] = '1'
# array[2][2][0] = '1'
# array[3][2][0] = '1'
# array[4][2][0] = '1'

''' This block generates a build that should succeed '''

# Adding 4x2x1 base (length x width x height)
array[1][0][1] = '1'
array[2][0][1] = '1'
array[3][0][1] = '1'
array[4][0][1] = '1'
array[1][0][2] = '1'
array[2][0][2] = '1'
array[3][0][2] = '1'
array[4][0][2] = '1'

# Adding 1x1x2 block (length x width x height) on top of base
array[2][1][1] = '1'
array[2][2][1] = '1'

# Adding 4x1x1 block on top of 1x1x2 block
array[2][3][1] = '1'
array[3][3][1] = '1'
array[4][3][1] = '1'
array[5][3][1] = '1'

########################################################################


# for 2D space
def in_hull(p, hull):
    if not isinstance(hull, Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p) >= 0

# def in_hull(points, x):
#     n_points = len(points)
#     n_dim = len(x)
#     c = numpy.zeros(n_points)
#     a = numpy.r_[points.T, numpy.ones((1, n_points))]
#     b = numpy.r_[x, numpy.ones(1)]
#     lp = linprog(c, A_eq=a, b_eq=b)
#     return lp.success


# If COM coordinates fall within support polygon, structure is stable
def in_support_polygon(point):
    # points = numpy.random.randint(1, 10, size=(6, 3))   # 6 random points in 2-D X-Z space
    points = numpy.column_stack((support_list_x, support_list_z))

    if (len(points)) >= 3:
        # Three following lines are for giving visual representation of convex hull of a point cloud
        # Can be commented out in ML project
        hull = ConvexHull(points)
        for simplex in hull.simplices:
            plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
        plt.show()

        # Pass in point and point cloud to in_hull function for Delaunay triangulation
        return in_hull([point], points)
    else:
        for item in points:
            if item.tolist() == point:
                return True
        return False


def calc_center_of_mass(list_param):
    temp_num = 0  # for temporary calculations for equations
    temp_dem = 0  # for temporary calculations for equations
    for item in list_param:
        temp_num += mass*item
        temp_dem += mass
        # print(item)
    return temp_num / temp_dem


def find_center_of_mass(array_param):
    global x_com, y_com, z_com, mass_list_x, mass_list_y, mass_list_z, support_list_x, support_list_z
    count = 0

    for x in range(len(array_param)):
        for y in range(len(array_param[x])):
            for z in range(len(array_param[x, y])):
                if array_param[x, y, z] == 1:
                    # print(x, y, z)
                    count += 1  # coordinates occupied
                    mass_list_x.append(x)
                    mass_list_y.append(y)
                    mass_list_z.append(z)
                    if y == 0:
                        support_list_x.append(x)
                        support_list_z.append(z)

    x_com = calc_center_of_mass(mass_list_x)
    y_com = calc_center_of_mass(mass_list_y)
    z_com = calc_center_of_mass(mass_list_z)
    print('Center of Mass X: %f' % x_com)
    print('Center of Mass Y: %f' % y_com)
    print('Center of Mass Z: %f' % z_com)
    return


def calculate_stability(array_param):
    find_center_of_mass(array_param)
    return


def findAdjacentNodes(array_param, coordinate):
    neighbours = set()
    # print(coordinate)
    # list(coordinate)[0]
    x = int(coordinate[0])
    y = int(coordinate[1])
    z = int(coordinate[2])
    print(array_param)

    try:
        if array_param[x + 1][y][z] == 1:
            neighbours.add(str(x+1) + str(y) + str(z))
        if array_param[x - 1][y][z] == 1:
            neighbours.add(str(x-1) + str(y) + str(z))
        if array_param[x][y][z + 1] == 1:
            neighbours.add(str(x) + str(y) + str(z+1))
        if array_param[x][y][z - 1] == 1:
            neighbours.add(str(x) + str(y) + str(z-1))
        if array_param[x][y + 1][z] == 1:
            neighbours.add(str(x) + str(y+1) + str(z))
        return neighbours
    except:
        return set()


# Graph for detecting multiple structures with Dijkstra's algorithm
graph = {}


def recursive_search(array_param, coordinate):
    global graph
    if coordinate not in graph:
        # print(coordinate)
        temp = findAdjacentNodes(array_param, coordinate)
        # print(temp)
        if len(temp) == 0:
            return
        else:
            graph[coordinate] = temp
            # print(graph)
            for item in graph[coordinate]:
                recursive_search(array_param, item)
    else:
        return


def findStructures(array_param):
    for x in range(len(array_param)):
        for y in range(len(array_param[x])):
            if y == 0:
                for z in range(len(array_param[x, y])):
                    print('Run recursion function on this coordinate')

    # parse graph
    for x in range(len(array_param)):
        for y in range(len(array_param[x])):
            if y == 0:
                for z in range(len(array_param[x, y])):
                    print('Recursively check if every occupied x-z coordinate can go to another occupied x-z coordinate')
                   
    return


# Running code
# print(findStructures(array))
recursive_search(array, '100')
print(graph)

# calculate_stability(array)
# if in_support_polygon([x_com, z_com]):
#     print('Point (%f, %f) is inside the support polygon' % (x_com, z_com))
# else:
#     print('Point (%f, %f) is not inside the support polygon' % (x_com, z_com))
