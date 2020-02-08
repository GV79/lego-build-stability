import numpy
from scipy.spatial import ConvexHull, Delaunay
import matplotlib.pyplot as plt

# fix edge case where dimensions > 1 digit

# 2D array of structures
structures_list = []

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
array = numpy.zeros((10, 10, 6))

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

# # Adding 1x1x5 block
# array[1][0][0] = '1'
# array[1][1][0] = '1'
# array[1][2][0] = '1'
# array[1][3][0] = '1'
# array[1][4][0] = '1'
#
# # Adding 1x2x1 block on top
# array[1][5][0] = '1'
# array[2][5][0] = '1'
# array[3][5][0] = '1'

''' This block generates a build that should pass stability check: Scenario 3 '''

# Adding 1x4x1 block
array[0][0][1] = '1'
array[1][0][1] = '1'
array[2][0][1] = '1'
array[3][0][1] = '1'

# Adding 1x4x1 block on top
array[1][1][1] = '1'
array[2][1][1] = '1'
array[3][1][1] = '1'
array[4][1][1] = '1'

# # Adding 1x2x3 block on top
array[8][0][2] = '1'
array[9][0][2] = '1'
array[8][1][2] = '1'
array[9][1][2] = '1'
array[8][2][2] = '1'
array[9][2][2] = '1'
array[8][3][2] = '1'
array[9][3][2] = '1'
array[8][4][2] = '1'
array[9][4][2] = '1'

# # Adding separate structure


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
    structures = find_structures(array_param)
    # print('%d structure(s) were found within the array' % find_structures(array_param))
    if structures == 1:
        find_center_of_mass(array_param)
        if in_support_polygon([x_com, z_com]):
            print('Point (%f, %f) is inside the support polygon' % (x_com, z_com))
            print('Therefore, the structure is stable.')
        else:
            print('Point (%f, %f) is not inside the support polygon' % (x_com, z_com))
            print('Therefore, the structure is unstable.')
    else:
        print(structures_list)
        # for i in structures_list:
        #     print(i)
        #     for j in i:
        #         print(j)
        print('%d structure(s) were found within the array' % structures)
    return


# Function that finds neighbouring coordinates
def find_adjacent_nodes(array_param, coordinate):
    coord_parsed = coordinate.split("-")
    if array_param[int(coord_parsed[0]), int(coord_parsed[1]), int(coord_parsed[2])] == 0:
        return []

    neighbours = set()
    x = int(coord_parsed[0])
    y = int(coord_parsed[1])
    z = int( coord_parsed[2])

    try:
        if array_param[x + 1][y][z] == 1:
            neighbours.add(str(x+1) + "-" + str(y) + "-" + str(z))
        if array_param[x - 1][y][z] == 1:
            neighbours.add(str(x-1) + "-" + str(y) + "-" + str(z))
        if array_param[x][y][z + 1] == 1:
            neighbours.add(str(x) + "-" + str(y) + "-" + str(z+1))
        if array_param[x][y][z - 1] == 1:
            neighbours.add(str(x) + "-" + str(y) + "-" + str(z-1))
        if array_param[x][y + 1][z] == 1:
            neighbours.add(str(x) + "-" + str(y+1) + "-" + str(z))
        return neighbours
    except:
        return set()


# Dictionary for detecting multiple structures
graph = {}


# Creating global object with all paths
def recursive_search(array_param, coordinate):
    global graph
    if coordinate not in graph:
        temp = find_adjacent_nodes(array_param, coordinate)
        coord_parsed = coordinate.split("-")
        if len(temp) == 0 or array_param[int(coord_parsed[0]), int(coord_parsed[1]), int(coord_parsed[2])] == 0:
            return
        else:
            graph[coordinate] = temp
            for item in graph[coordinate]:
                recursive_search(array_param, item)
    else:
        return


# Recursive search that returns an array of connections for a given coord
def recursive_search_item(array_param, coordinate, temp_dict):
    if coordinate not in temp_dict:
        temp = find_adjacent_nodes(array_param, coordinate)
        coord_parsed = coordinate.split("-")
        if len(temp) == 0 or array_param[int(coord_parsed[0]), int(coord_parsed[1]), int(coord_parsed[2])] == 0:
            return temp_dict
        else:
            temp_dict[coordinate] = temp
            for item in temp_dict[coordinate]:
                recursive_search_item(array_param, item, temp_dict)
            return temp_dict
    else:
        return temp_dict


# Looks for multiple structures within 3D array
def find_structures(array_param):
    structures = 1
    occupied_x_z_list = list()
    blacklist = dict()
    # for coordinates that have been counted as part of a structure, can ignore all nodes directly linked

    for x in range(len(array_param)):
        for y in range(len(array_param[x])):
            if y == 0:
                for z in range(len(array_param[x, y])):
                    if array_param[x, y, z] == 1:
                        # '-' is delimiter to avoid edge case where digits > 1
                        recursive_search(array, str(x)+"-"+str(y)+"-"+str(z))
                        occupied_x_z_list.append([x, y, z])
    if len(graph) == 0:
        return 0

    if len(array_param) * len(array_param[x, y]) == len(occupied_x_z_list):
        return 1

    for i in occupied_x_z_list:
        coord_string_one = str(i[0]) + "-" + str(i[1]) + "-" + str(i[2])
        for j in occupied_x_z_list:
            coord_string_two = str(j[0]) + "-" + str(j[1]) + "-" + str(j[2])
            if coord_string_one in blacklist or coord_string_two in blacklist:
                continue
            elif i != j:
                if find_shortest_path(graph, coord_string_one, coord_string_two, []) is None:
                    temp_dict_one = recursive_search_item(array_param, coord_string_one, {})
                    temp_dict_two = recursive_search_item(array_param, coord_string_two, {})
                    blacklist.update(temp_dict_one)
                    blacklist.update(temp_dict_two)
                    structures_list.append(temp_dict_one.keys())
                    structures_list.append(temp_dict_two.keys())
                    structures += 1
    return structures


def find_shortest_path(graph_param, start, end, path=[]):
    path = path + [start]
    if start == end:
        return path
    if start not in graph_param:
        return None
    shortest = None
    for node in graph_param[start]:
        if node not in path:
            new_path = find_shortest_path(graph_param, node, end, path)
            if new_path:
                if not shortest or len(new_path) < len(shortest):
                    shortest = new_path
    return shortest


''' Executing code'''
calculate_stability(array)
