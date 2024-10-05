# Task 1: Define Graph Representation
class Graph:
    def __init__(self):
        self.vertices = {}
    
    def add_vertex(self, vertex):
        """Adds a vertex if it doesn't already exist."""
        if vertex not in self.vertices:
            self.vertices[vertex] = {}
    
    def add_age(self, source, destination, weight):
        """Adds an directed adge between two vertices with a specisfied weight."""
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight
    def get_neighbors(self, vertex):
        """Returns the neighbors and edge weights for the given vertex."""
        if vertex in self.vertices:
            return self.vertices[vertex]
        else:
            return {}
        
# Example useage
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_edge('A', 'B', 5)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)

print(graph.vertices)
# Output: {'A': {'B': 5, 'C': 10}, 'B': {'A': 5, 'C': 3}, 'C': {'B': 3, 'A': 10}}

# Task 2: Implement Dijkstra's Algorithm
import heapq

class Graph:
    def __init__(self):
        self.vertices = {}
    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}
    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight # For undirected graph
    def get_neighbores(self, vertex):
        return self.vertices.get(vertex, {})
    
    def dijkstra(self, start_vertex):
        # Min-heap priority queue, initialized with start vertex (distance 0)
        priority_queue = [(0, start_vertex)]
        distances = {vertex: float('infinity') for vertex in self.vertices} # Set all distances to infinity
        distances[start_vertex] = 0 # Distance to start vertex is 0
        previous_nodes = {vertex: None for vertex in self.vertices} # For path reconstaraction 

        while priority_queue:
            current_distance, current_vertex = heapq.heappop(priority_queue)

            # If a popped vertex has a larger disance than the recorded, skip it 
            if current_distance > distances[current_vertex]:
                continue

            # Explore neighbors
            for neighbor, weight in self.get_neighbors(current_vertex).items():
                distance = current_distance + weight

                # If a shorter path to the neighbor is found
                if distance < distances[neighbor]:
                    distances[neighbor] = distance  # Update shortest distance
                    previous_nodes[neighbor] = current_vertex # Track path
                    heapq.heappush(priority_queue, (distance, neighbor)) # Add to queue
        
        return distances, previous_nodes
    def reconstruct_path(self, start_vertex, end_vertex, previous_nodes):
        path = []
        current_node = end_vertex

        while current_node is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]
        path.reverse() # Reverse to get path from start to end
        if path[0] == start_vertex: # Ensure path starts with the start vertex
            return path
        else:
             return [] # No path exists

# Example useage
graph = Graph()
graph.add_vertext('A')
graph.add_vertext('B')
graph.add_vertext('C')
graph.add_vertext('D')
graph.add_vertext('E')

graph.add.edge('A', 'B', 1)
graph.add.edge('A', 'C', 4)
graph.add.edge('B', 'C', 2)
graph.add.edge('B', 'D', 5)
graph.add.edge('C', 'E', 1)
graph.add.edge('D', 'E', 3)

# Run Dijkstra's algorithm from vertex 'A'
distances, previous_nodes = graph.dijkstra('A')

# Shortest distances from 'A' to all other vertices
print("Shortest distances: ", distances)

# Reconstruct the path from 'A' to 'E'
path = graph.reconstruct_path('A', 'E', previous_nodes)
print("Shortest path from A to E: ", path)


# Task 3: Test the Algorithm Implementation
import heapq
def dijkstra(graph, source):
    """
    Implements Dijkstra's algorithm to find the shortest paths from a source nodes in a graph.

    Args:
        graph: A dictionary representing the graph, where keys are nodes and values are dictionadjearies 
    contianing adjacent nodes and thier corresponding weights.
        source: The source nodes from which to calculate shortest paths.
    Returns:
        A dictionary containing the shortest distances from the source node to all other nodes.
    """
    distances = {node: float('inf') for node in graph }
    distances[source] = 0

    priority_queue = [(0, source)]

    while priority_queue:
        current_distance, current_node, = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue
        for neighbor, weight in graph[current_node].items():
            new_distance = current_distance + weight
            if new_distance < distances[neighbor]:

                distances[neighbor] = new_distance
                heapq.heappush(priority_queue, (new_distance, neighbor))
    return distances

# Example useage:
graph = {
    'A': {'B': 4, 'C': 3},
    'B': {'D': 2},
    'C': {'D': 5, 'E': 1},
    'D': {'E': 3},
    'E': {}
}

source_node = 'A'
shortest_distances = dijkstra(graph, source_node)
print(shortest_distances)


# Task 4: Analyze Time and Space Complexity
import heapq
def dijkstra(graph, source):
     """
    Implements Dijkstra's algorithm to find the shortest paths from a sources vertex to all other vertices in a graph.
    
    Args:
        graph: A dictionary representing the graph, where keys are vertices and values are lists of tuples  (neighbor, weight).
        source: The source vertex.
    Returns:
        A dictionary containing the shortest distances from the vertex  to all other vertices.
    """
     
     distances = {vertex: float('inf') for vertex in graph}
     distances[source] = 0
     priority_queue = [(0, source)]

     while priority_queue:
         current_distance, current_vertex = heapq.heappop(priority_queue)
         if current_distance > distances[current_vertex]:
             continue # Skip if a shorter path has already been found 
         for neighbor, weight in graph[current_vertex]:
             alternative_distance = current_distance + weight
             if alternative_distance < distances[neighbor]:
                 distances[neighbor] = alternative_distance
                 heapq.heappush(priority_queue, (alternative_distance, neighbor))
     return distances
# Example useage:
graph = {
    'A': [('B, 4'), ('C', 3)],
    'B': [('D', 2)],
    'C': [('D', 5)],
    'D': []
}

source_vertex = 'A'
shortest_distances = dijkstra(graph, source_vertex)

print("Shortest distances from", source_vertex, "to other vertices:")
for vertex, distance in shortest_distances.items():
    print(vertex, ":", distance)


         







