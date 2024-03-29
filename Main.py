import heapq  # Import heapq module for priority queue operations

def dijkstra(graph, start):
    # Initialize distances to all nodes as infinity
    distances = {node: float('inf') for node in graph}
    distances[start] = 0  # Distance to the starting node is set to 0
    queue = [(0, start)]  # Initialize a priority queue with the start node set distance to 0

    while queue:  # Iterate until the priority queue is empty
        current_distance, current_node = heapq.heappop(queue) # Get node with the least distance

        if current_distance > distances[current_node]:
            continue  # Skip this if a shorter path to the current node is already found

        # Iterate through the neighbors of the current node
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight  # Calculate the distance to the neighbor
            if distance < distances[neighbor]:  
                distances[neighbor] = distance  # Update the distance to the neighbor
                heapq.heappush(queue, (distance, neighbor)) # Add neighbor to the priority queue

    return distances  # Return the calculated distances

def shortest_paths_to_charging_stations(graph, charging_stations, start_node):
    distances = dijkstra(graph, start_node)  # Calculate distances using Dijkstra's algorithm
    shortest_path = None  # Initialize shortest path as None
    shortest_distance = float('inf')  # Initialize shortest distance as infinity

    # Iterate through each charging station
    for station in charging_stations:
        if distances[station] < shortest_distance: 
            shortest_distance = distances[station]  # Update shortest distance
            shortest_path = station  # Update shortest path

    return shortest_path, shortest_distance  # Return the shortest path and distance

if __name__ == "__main__":
    # Defines the graph representing the network
    graph = {
        'A': [('B', 6), ('F', 5)],
        'B': [('A', 6), ('C', 5), ('G', 6)],
        'C': [('B', 5), ('D', 7), ('H', 5)],
        'D': [('C', 7), ('E', 7), ('I', 8)],
        'E': [('D', 7), ('I', 6), ('N', 15)],
        'F': [('A', 5), ('G', 8), ('J', 7)],
        'G': [('B', 6), ('F', 8), ('H', 9), ('K', 8)],
        'H': [('C', 5), ('G', 9), ('I', 12)],
        'I': [('D', 8), ('E', 6), ('H', 12), ('M', 10)],
        'J': [('F', 7), ('K', 5), ('O', 7)],
        'K': [('G', 8), ('J', 5), ('L', 7)],
        'L': [('K', 7), ('M', 7), ('P', 7)],
        'M': [('I', 10), ('L', 7), ('N', 9)],
        'N': [('E', 15), ('M', 9), ('R', 7)],
        'O': [('J', 7), ('P', 13), ('S', 9)],
        'P': [('L', 7), ('O', 13), ('U', 11)],
        'Q': [('R', 9)],
        'R': [('N', 7), ('Q', 9), ('W', 10)],
        'S': [('O', 9), ('T', 9)],
        'T': [('S', 9), ('U', 8)],
        'U': [('P', 11), ('T', 8), ('V', 8)],
        'V': [('U', 8), ('W', 5)],
        'W': [('R', 10), ('V', 5)]
    }

    charging_stations = ['H', 'K', 'Q', 'T']  # List of charging stations

    start_node = input("Enter the starting node: ").upper()  #Accept start node from user input

    if start_node not in graph:  # Check if start node is valid
        print("Invalid start node!")
    else:
        shortest_path, shortest_distance = shortest_paths_to_charging_stations(graph, charging_stations, start_node)

        if shortest_path:
            print(f"Shortest path to the nearest charging station: {shortest_path}")
            print(f"Shortest distance: {shortest_distance}")

            # Calculate distances to all charging stations
            all_distances = dijkstra(graph, start_node)
            for station in charging_stations:
                print(f"Distance to charging station {station}: {all_distances[station]}")
        else:
            print("No reachable charging station.")
