import numpy
from scipy.spatial import ConvexHull, Delaunay
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
# Might be better to just combine these two into a support_set
support_list_x = []
support_list_z = []

# Standardized unit of mass
mass = 1

# Initializing example 3D input function would receive (params x=6,y=10,z=6)
array = numpy.zeros((6, 10, 6))

#################################################################################
''' Check /scenarioImages for visual representation in Lego Studio (LEGO CAD) '''
''' All scenarios confirmed using stability feature in Lego Studio (LEGO CAD) '''
''' This block generates a build that should fail stability check: Scenario 1 '''

# # Adding 1x1x2 block (width x length x height)
# array[1][0][0] = '1'
# array[1][1][0] = '1'
#
# # Adding 1x4x1 block on top of 1x1x2 block
# array[1][2][0] = '1'
# array[2][2][0] = '1'
# array[3][2][0] = '1'
# array[4][2][0] = '1'

''' This block generates a build that should pass stability check: Scenario 2 '''

# # Adding 2x4x1 base (width x length x height)
# array[1][0][1] = '1'
# array[2][0][1] = '1'
# array[3][0][1] = '1'
# array[4][0][1] = '1'
# array[1][0][2] = '1'
# array[2][0][2] = '1'
# array[3][0][2] = '1'
# array[4][0][2] = '1'
#
# # Adding 1x1x2 block (width x length x height) on top of base
# array[2][1][1] = '1'
# array[2][2][1] = '1'
#
# # Adding 1x4x1 block on top of 1x1x2 block
# array[2][3][1] = '1'
# array[3][3][1] = '1'
# array[4][3][1] = '1'
# array[5][3][1] = '1'

''' This block generates a build that should pass stability check: Scenario 3 '''

# Adding 1x1x5 block
array[1][0][0] = '1'
array[1][1][0] = '1'
array[1][2][0] = '1'
array[1][3][0] = '1'
array[1][4][0] = '1'

# Adding 1x2x1 block on top
array[1][5][0] = '1'
array[2][5][0] = '1'
array[3][5][0] = '1'

#################################################################################


# Function that uses Delaunay Triangulation to determine whether a point exists in the hull
def in_hull(p, hull):
    if not isinstance(hull, Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p) >= 0


# Function that determines whether build is stable with param 'point' - xCOM, zCOM
# Based on whether x_com and z_com COM values fall within support polygon
def in_support_polygon(point):
    support_polygon_vertices = list()  # array containing x-z coordinates
    # points = numpy.random.randint(1, 10, size=(6, 3))   # generate 6 random points in 2-D space
    points = numpy.column_stack((support_list_x, support_list_z))  # X-Z coordinate that are y=0

    hull = ConvexHull(points)
    for item in hull.vertices:
        support_polygon_vertices.append(points[item])
    print(support_polygon_vertices)

    # 3 following lines can be commented out when used in actual learning process
    for simplex in hull.simplices:
        plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
    plt.show()  # gives visual representation of convex hull of a point cloud

    return in_hull([point], points)  # Pass in point and point cloud to in_hull function


# Function that uses center of mass equation
def calc_center_of_mass(list_param):
    temp_num = 0  # for temporary calculations for equations
    temp_dem = 0  # for temporary calculations for equations
    for item in list_param:
        temp_num += mass*item
        temp_dem += mass
    return temp_num / temp_dem


# Function that assigns center of mass x,y,z values to global vars
def find_center_of_mass(array_param):
    global x_com, y_com, z_com, mass_list_x, mass_list_y, mass_list_z, support_list_x, support_list_z

    for x in range(len(array_param)):
        for y in range(len(array_param[x])):
            for z in range(len(array_param[x, y])):
                if array_param[x, y, z] == 1:
                    mass_list_x.append(x+0.5)
                    mass_list_y.append(y+0.5)
                    mass_list_z.append(z+0.5)
                    if y == 0:
                        # Each point has a boundary of 4 vertices
                        support_list_x.append(x)
                        support_list_z.append(z)
                        support_list_x.append(x+1)
                        support_list_z.append(z)
                        support_list_x.append(x)
                        support_list_z.append(z+1)
                        support_list_x.append(x+1)
                        support_list_z.append(z+1)

    x_com = calc_center_of_mass(mass_list_x)
    y_com = calc_center_of_mass(mass_list_y)
    z_com = calc_center_of_mass(mass_list_z)
    print('Center of Mass X: %f' % x_com)
    print('Center of Mass Y: %f' % y_com)
    print('Center of Mass Z: %f' % z_com)
    return


# Function that will execute multiple structure searching and then individual COM calculations
def calculate_stability(array_param):
    find_center_of_mass(array_param)
    return


# Function that finds neighbouring coordinates
def find_adjacent_nodes(array_param, coordinate):
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


# Dictionary for detecting multiple structures
graph = {}


def recursive_search(array_param, coordinate):
    global graph
    if coordinate not in graph:
        temp = find_adjacent_nodes(array_param, coordinate)
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


def find_structures(array_param):
    occupied_x_z_list = list()
    for x in range(len(array_param)):
        for y in range(len(array_param[x])):
            if y == 0:
                for z in range(len(array_param[x, y])):
                    print('Run recursion search on this coordinate')
                    occupied_x_z_list.append([x, y, z])

    # print('Recursively check in list if every occupied x-z coordinate can go to another occupied x-z coordinate')
    return


def is_path_available(array_param, coord_start, coord_end):
    visited = set()
    visited.add(coord_start)
    for item in graph[coord_start]:
        if item == coord_end:
            return True
        elif item in visited:
            continue
        else:
            visited.add(item)
            return is_path_available(array_param, item, coord_end)
    return False


''' Executing code'''
# print(find_structures(array))
# recursive_search(array, '100')
# print(graph)

calculate_stability(array)
if in_support_polygon([x_com, z_com]):
    print('Point (%f, %f) is inside the support polygon' % (x_com, z_com))
    print('Therefore, the structure is stable.')
else:
    print('Point (%f, %f) is not inside the support polygon' % (x_com, z_com))
    print('Therefore, the structure is unstable.')
