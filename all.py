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
graph.add_edge('B', 'C', 5)
graph.add_edge('A', 'C', 5)

print(graph.vertices)
# Output: {'A': {'B': 5, 'C': 10}, 'B': {'A': 5, 'C': 3}, 'C': {'B': 3, 'A': 10}}