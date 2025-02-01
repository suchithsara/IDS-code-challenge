import numpy as np
from scipy.spatial.distance import euclidean
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def distance(p1, p2):
    return euclidean(p1, p2)

def tsp_nearest_neighbor(locations, priorities):
    priority_map = {'high': 1, 'medium': 2, 'low': 3}
    indexed_locations = sorted(zip(locations, priorities), key=lambda x: priority_map[x[1]])
    sorted_locations = [loc for loc, _ in indexed_locations]
    
    num_locations = len(sorted_locations)
    distance_matrix = [[distance(sorted_locations[i], sorted_locations[j]) for j in range(num_locations)] for i in range(num_locations)]
    
    manager = pywrapcp.RoutingIndexManager(num_locations, 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    
    solution = routing.SolveWithParameters(search_parameters)
    
    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(sorted_locations[manager.IndexToNode(index)])
            index = solution.Value(routing.NextVar(index))
        route.append(sorted_locations[0])
        
        total_distance = sum(distance(route[i], route[i+1]) for i in range(len(route)-1))
        return route, round(total_distance, 2)
    else:
        return [], float('inf')

# Dynamic User Inputs
locations = []
priorities = []
while True:
    try:
        n = int(input("Enter the number of locations: ").strip())
        if n <= 0:
            raise ValueError("Number of locations must be a positive integer.")
        break
    except ValueError as e:
        print(f"Invalid input: {e}. Please enter a valid positive integer.")

for i in range(n):
    while True:
        try:
            coords = input(f"Enter coordinates for location {i+1} (x y): ").strip().split()
            if len(coords) != 2:
                raise ValueError("Please enter exactly two integer values separated by a space.")
            x, y = map(int, coords)
            break
        except ValueError as e:
            print(f"Invalid input: {e}")
    
    while True:
        priority = input(f"Enter priority for location {i+1} (high/medium/low): ").strip().lower()
        if priority in {'high', 'medium', 'low'}:
            break
        else:
            print("Invalid priority. Please enter 'high', 'medium', or 'low'.")
    
    locations.append((x, y))
    priorities.append(priority)

optimized_route, total_distance = tsp_nearest_neighbor(locations, priorities)
print("Optimized Route:", optimized_route)
print("Total Distance:", total_distance, "units")
