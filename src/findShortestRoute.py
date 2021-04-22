from collections import defaultdict
  
class ShortestRoute:

    def __init__(self):
        # Subscribing to node that proccesses SLAM image to get list of vertices

        # Publishing shortest path from start to finish
        self.path_Pub = rospy.Publisher('/path_tree', array, queue_size=1)
  
  
    # Find vertex with minimum distance
    def minDistance(self,dist,queue):
        # Set min value and min_index as -1
        minimum = float("Inf")
        min_index = -1
        
        # Find min item with min val and is
        # still in queue
        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i]
                min_index = i
        return min_index
  
  
    # Print shortest path from source to j
    def printPath(self, parent, j):
          
        # If j = source
        if parent[j] == -1 : 
            print j,
            return
        self.printPath(parent , parent[j])
        print j,
          
    # Find path
    def dijkstra(self, graph, src):
  
        row = len(graph)
        col = len(graph[0])
  
        # output array
        dist = [float("Inf")] * row
  
        # Shortest path tree
        parent = [-1] * row
  
        # Distance from source to itself
        dist[src] = 0
      
        # Add all vertices in queue
        queue = []
        for i in range(row):
            queue.append(i)
              
        #Find shortest path for all nodes
        while queue:
  
            # Pick min dist vertex still in queue
            u = self.minDistance(dist,queue) 
  
            # remove min element     
            queue.remove(u)
  
            # Update dist and parent of adjacent vertices
            # of picked vertex from queue
            for i in range(col):
                # Update dist[i] if in queue, there is edge from u to i,
                # and total weight from source to i through u is smaller 
                # than current val of dist[i]
                if graph[u][i] and i in queue:
                    if dist[u] + graph[u][i] < dist[i]:
                        dist[i] = dist[u] + graph[u][i]
                        parent[i] = u
  
        # Use parent arr to get shortest path tree

if __name__ == "__main__":
    rospy.init_node('path_tree')
    mask_publisher = TriangleMask()
    g = ShortestRoute()

    # Sample values
    graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
            [4, 0, 8, 0, 0, 0, 0, 11, 0],
            [0, 8, 0, 7, 0, 4, 0, 0, 2],
            [0, 0, 7, 0, 9, 14, 0, 0, 0],
            [0, 0, 0, 9, 0, 10, 0, 0, 0],
            [0, 0, 4, 14, 10, 0, 2, 0, 0],
            [0, 0, 0, 0, 0, 2, 0, 1, 6],
            [8, 11, 0, 0, 0, 0, 1, 0, 7],
            [0, 0, 2, 0, 0, 0, 6, 7, 0]
            ]

    rospy.spin()