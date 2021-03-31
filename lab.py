#!/usr/bin/env python3

from util import read_osm_data, great_circle_distance, to_local_kml_url

# NO ADDITIONAL IMPORTS!


ALLOWED_HIGHWAY_TYPES = {
    'motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified',
    'residential', 'living_street', 'motorway_link', 'trunk_link',
    'primary_link', 'secondary_link', 'tertiary_link',
}


DEFAULT_SPEED_LIMIT_MPH = {
    'motorway': 60,
    'trunk': 45,
    'primary': 35,
    'secondary': 30,
    'residential': 25,
    'tertiary': 25,
    'unclassified': 25,
    'living_street': 10,
    'motorway_link': 30,
    'trunk_link': 30,
    'primary_link': 30,
    'secondary_link': 30,
    'tertiary_link': 25,
}


def build_auxiliary_structures(nodes_filename, ways_filename):
    """
    Create any auxiliary structures you are interested in, by reading the data
    from the given filenames (using read_osm_data)

    my structure - dictionary of nodes ids w/ their location, tags, and what nodes they are connected to by valid ways
        also has a 'locations' section that has (lat, lon) as the keys w/ values being the node id of the node w/ that location
    {
        n1_id: {
            lat: ( )
            lon: ( )
            tags: {}
            connected: dict of node ids of places you can get to from here w/ the speed limit as the value
        }
        ...

        'locations'
            (location tuples): node_id
            ...
    }
    """
    my_structure = {
        'locations': {}
    }
    connected_nodes = set()
    # loops over the ways in the file
    for way in read_osm_data(ways_filename):
        # checks if they are a valid way
        if 'highway' in way['tags'] and way['tags']['highway'] in ALLOWED_HIGHWAY_TYPES:
            # if it is, gets the speed limit of the way
            speed_limit = 0
            if 'maxspeed_mph' in way['tags']:
                speed_limit = way['tags']['maxspeed_mph']
            else:
                speed_limit = DEFAULT_SPEED_LIMIT_MPH[way['tags']['highway']]
            # loops over the ways nodes (front to back), and for each node pair (A, B) adds B to A's connected section w/ the speed limit as the value
            prev_node_id = None
            for node in way['nodes']:
                if prev_node_id:
                    if prev_node_id in my_structure:
                        my_structure[prev_node_id]['connected'][node] = speed_limit
                    else:
                        my_structure[prev_node_id] = {
                            'connected': {
                                node: speed_limit
                            }
                        }
                connected_nodes.add(node)
                prev_node_id = node
            # if the road is two-way, it loops over the ways nodes in reverse (back to front), and for each node pair (B, A) adds A to B's connected section w/ the speed limit as the value
            if not ('oneway' in way['tags'] and way['tags']['oneway'] == 'yes'):
                # two way road
                prev_node_id = None
                for node in reversed(way['nodes']):
                    if prev_node_id:
                        if prev_node_id in my_structure:
                            my_structure[prev_node_id]['connected'][node] = speed_limit
                        else:
                            my_structure[prev_node_id] = {
                                'connected': {
                                    node: speed_limit
                                }
                            }
                    connected_nodes.add(node)
                    prev_node_id = node
    # loops over all the nodes in the file
    for node in read_osm_data(nodes_filename):
        # if the node is connected to something, then it adds its lat, lon, and tags information to the refactored structure
        if node['id'] in connected_nodes:
            if node['id'] in my_structure:
                my_structure[node['id']].update({
                    'lat': node['lat'],
                    'lon': node['lon'],
                    'tags': node['tags'].copy()
                })
            else:
                my_structure[node['id']] = {
                    'lat': node['lat'],
                    'lon': node['lon'],
                    'tags': node['tags'].copy(),
                    'connected': {}
                }
            my_structure['locations'][(node['lat'], node['lon'])] = node['id']
    return my_structure

def get_lat_lon(node_id, aux_structures):
    """ Helper function
    Returns a 2-element tuples with the lat and lon of the node with the inputed node_id
    """
    if node_id in aux_structures:
        return (aux_structures[node_id]['lat'], aux_structures[node_id]['lon'])
    return None

def uniform_cost_search(start, goal, neighbors, cost, aux_structures, heuristic=False):
    # Initialize an "agenda" (containing paths to consider, as well as their costs).
    agenda = [([ start ], 0)] # holds 2-element tuple w/ path and cost
    # Intialize empty "expanded" set (set of vertices we've ever removed from the agenda)
    expanded_set = set()
    # num_of_paths_popped = 0 # used to track the difference in pops w/ and w/o heuristic
    while agenda:
        path = None
        min_cost = None
        min_index = 0
        index = 0
        # Remove the path with the lowest cost from the agenda.
        for path_and_cost in agenda:
            if path_and_cost:
                if heuristic:
                    g = path_and_cost[1] # path cost from the starting node to node n
                    h = great_circle_distance(get_lat_lon(path_and_cost[0][-1], aux_structures), get_lat_lon(goal, aux_structures)) # estimated cost of the lowest-cost path from node n to the goal node
                    f = g + h # estimated cost of the lowest-cost solution involving n
                    if min_cost == None or f < min_cost:
                        min_cost = f
                        min_index = index
                else:
                    if min_cost == None or path_and_cost[1] < min_cost:
                        min_cost = path_and_cost[1]
                        min_index = index
            index += 1
        # num_of_paths_popped += 1
        path = agenda[min_index]
        del agenda[min_index]
        # If this path's terminal vertex is in the expanded set, ignore it completely and move on to the next path.
        terminal_vertex = path[0][-1]
        if terminal_vertex in expanded_set:
            continue
        # If this path's terminal vertex satisfies the goal condition, return that path (hooray!). Otherwise, add its terminal vertex to the expanded set.
        if terminal_vertex == goal:
            # print(num_of_paths_popped)
            return path[0]
        expanded_set.add(terminal_vertex)
        # For each of the children of that path's terminal vertex:
        for n in neighbors(terminal_vertex):
            # If it is in the expanded set, skip it
            if not n in expanded_set:
                c = cost(terminal_vertex, n)
                if c:
                    # Otherwise, add the associated path (and cost) to the agenda
                    agenda.append((path[0] + [n], path[1] + c))
        # until the agenda is empty (search failed)
    return None

def find_short_path_nodes(aux_structures, node1, node2):
    """
    Return the shortest path between the two nodes

    Parameters:
        aux_structures: the result of calling build_auxiliary_structures
        node1: node representing the start location
        node2: node representing the end location

    Returns:
        a list of node IDs representing the shortest path (in terms of
        distance) from node1 to node2
    """
    if node1 == node2:
        return [node1]
    def neighbors(node_id):
        if aux_structures[node_id]['connected'].keys():
            return aux_structures[node_id]['connected'].keys()
        return []
    def cost(node_id_1, node_id_2):
        loc1 = get_lat_lon(node_id_1, aux_structures)
        loc2 = get_lat_lon(node_id_2, aux_structures)
        if loc1 and loc2:
            return great_circle_distance(loc1, loc2)
        return None
    return uniform_cost_search(node1, node2, neighbors, cost, aux_structures, True)

def get_nearest_node_id(loc, aux_structures):
    """ helper function 
    Returns the id of closest node to this location 
    """
    min_dist = None
    min_node_id = None
    for l in aux_structures['locations']:
        dist = great_circle_distance(loc, l)
        if min_dist == None or min_dist > dist:
            min_dist = dist
            min_node_id = aux_structures['locations'][l]
    return min_node_id

def convert_nodes_path_to_loc_path(nodes_path, aux_structures):
    """ helper function
    Takes in a list of node ids and returns a list of 2-element (lat, lon) tuples
    """
    if nodes_path:
        loc_path = []
        for node_id in nodes_path:
            loc_path.append((aux_structures[node_id]['lat'], aux_structures[node_id]['lon']))
        return loc_path
    return None

def find_short_path(aux_structures, loc1, loc2):
    """
    Return the shortest path between the two locations

    Parameters:
        aux_structures: the result of calling build_auxiliary_structures
        loc1: tuple of 2 floats: (latitude, longitude), representing the start
              location
        loc2: tuple of 2 floats: (latitude, longitude), representing the end
              location

    Returns:
        a list of (latitude, longitude) tuples representing the shortest path
        (in terms of distance) from loc1 to loc2.
    """
    n1 = get_nearest_node_id(loc1, aux_structures)
    n2 = get_nearest_node_id(loc2, aux_structures)
    if not n1 or not n2:
        return None
    if n1 == n2:
        return [(aux_structures[n1]['lat'], aux_structures[n1]['lon'])]
    def neighbors(node_id):
        if aux_structures[node_id]['connected'].keys():
            return aux_structures[node_id]['connected'].keys()
        return []
    def cost(node_id_1, node_id_2):
        loc1 = get_lat_lon(node_id_1, aux_structures)
        loc2 = get_lat_lon(node_id_2, aux_structures)
        if loc1 and loc2:
            return great_circle_distance(loc1, loc2)
        return None
    nodes_path = uniform_cost_search(n1, n2, neighbors, cost, aux_structures, True)
    return convert_nodes_path_to_loc_path(nodes_path, aux_structures)


def find_fast_path(aux_structures, loc1, loc2):
    """
    Return the shortest path between the two locations, in terms of expected
    time (taking into account speed limits).

    Parameters:
        aux_structures: the result of calling build_auxiliary_structures
        loc1: tuple of 2 floats: (latitude, longitude), representing the start
              location
        loc2: tuple of 2 floats: (latitude, longitude), representing the end
              location

    Returns:
        a list of (latitude, longitude) tuples representing the shortest path
        (in terms of time) from loc1 to loc2.
    """
    n1 = get_nearest_node_id(loc1, aux_structures)
    n2 = get_nearest_node_id(loc2, aux_structures)
    if not n1 or not n2:
        return None
    if n1 == n2:
        return [(aux_structures[n1]['lat'], aux_structures[n1]['lon'])]
    def neighbors(node_id):
        if aux_structures[node_id]['connected'].keys():
            return aux_structures[node_id]['connected'].keys()
        return []
    def cost(node_id_1, node_id_2):
        loc1 = get_lat_lon(node_id_1, aux_structures)
        loc2 = get_lat_lon(node_id_2, aux_structures)
        speed_limit = aux_structures[node_id_1]['connected'][node_id_2]
        if loc1 and loc2:
            return great_circle_distance(loc1, loc2) / speed_limit
        return None
    nodes_path = uniform_cost_search(n1, n2, neighbors, cost, aux_structures, False)
    return convert_nodes_path_to_loc_path(nodes_path, aux_structures)

def get_node_by_id(node_id, dataset_name):
    """ Helper function 
    Returns node with the node id in the dataset
    """
    for node in read_osm_data('resources/' + dataset_name + '.nodes'):
        if node['id'] == node_id:
            return node

if __name__ == '__main__':
    # additional code here will be run only when lab.py is invoked directly
    # (not when imported from test.py), so this is a good place to put code
    # used, for example, to generate the results for the online questions.

    ### --- SECTION 2 --- ###
    # total_node_count = 0
    # count_with_name = 0
    # for node in read_osm_data('resources/cambridge.nodes'):
    #     # total_count += 1
    #     if 'name' in node['tags']:
    #         count_with_name += 1
    #         if node['tags']['name'] == '77 Massachusetts Ave':
    #             print(node['id'])
    #             break
    # print(total_node_count)
    # print(count_with_name)

    # total_way_count = 0
    # total_one_way_roads = 0
    # for way in read_osm_data('resources/cambridge.ways'):
        # total_way_count += 1
        # if 'oneway' in way['tags'] and way['tags']['oneway'] == 'yes':
        #     total_one_way_roads += 1
    # print(total_way_count)
    # print(total_one_way_roads)

    ### --- SECTION 3 --- ###
    # print(great_circle_distance((42.363745, -71.100999), (42.361283, -71.239677)))
    # loc1 = ()
    # loc2 = ()
    # for node in read_osm_data('resources/midwest.nodes'):
    #     if node['id'] == 233941454:
    #         loc1 = (node['lat'], node['lon'])
    #     if node['id'] == 233947199:
    #         loc2 = (node['lat'], node['lon'])
    #     if loc1 and loc2:
    #         print(great_circle_distance(loc1, loc2))
    #         break
    # road = {}
    # for way in read_osm_data('resources/midwest.ways'):
    #     if way['id'] == 21705939:
    #         road = way
    #         break
    # total_distance = 0
    # for index in range(len(road['nodes'])-1):
    #     node1 = get_node_by_id(road['nodes'][index], 'midwest')
    #     node2 = get_node_by_id(road['nodes'][index+1], 'midwest')
    #     total_distance += great_circle_distance((node1['lat'], node1['lon']), (node2['lat'], node2['lon']))
    # print(total_distance)

    # print(build_auxiliary_structures('./resources/mit.nodes', './resources/mit.ways'))
    # print(find_short_path_nodes(build_auxiliary_structures('./resources/mit.nodes', './resources/mit.ways'), 2, 8))

    ### --- SECTION 4 --- ###
    # structure_midwest = build_auxiliary_structures('./resources/midwest.nodes', './resources/midwest.ways')
    # print(get_nearest_node_id((41.4452463, -89.3161394), structure_midwest))

    ### --- SECTION 5 --- ###
    # structure_cambridge = build_auxiliary_structures('./resources/cambridge.nodes', './resources/cambridge.ways')
    # find_short_path(structure_cambridge, (42.3858, -71.0783), (42.5465, -71.1787))
    # the total number of paths we pull off of the agenda was
        # 386255 without heuristic
        # 47609 with heuristic 

    ### --- OTHER TESTING --- ###
    # structure = build_auxiliary_structures('./resources/mit.nodes', './resources/mit.ways')
    # for node_id in structure:
    #     print(node_id, ":", structure[node_id])
    # start_time = time.time()
    # cambridge = build_auxiliary_structures('./resources/cambridge.nodes', './resources/cambridge.ways')
    # print(time.time() - start_time, "sec")
    # cambridge - 17.162186861038208 seconds

    waltham = (42.3722, -71.2408)
    salem = (42.5195, -70.8967)
    aux = build_auxiliary_structures('resources/cambridge.nodes', 'resources/cambridge.ways')
    print(to_local_kml_url(find_short_path(aux, waltham, salem)))
    print(to_local_kml_url(find_fast_path(aux, waltham, salem)))

    pass
