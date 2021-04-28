import cv2
import numpy as np

#Helper functions and classes
class Vertex:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.distFromSrc = float('inf') 
        self.xParent = None
        self.yParent = None
        self.processed = False
        self.index_in_queue = None

#Return neighbor directly above, below, right, and left
def get_neighbors(graph,row,col):
    shape = graph.shape
    neighbors=[]
    #ensure neighbors are within image boundaries
    if row > 0 and not graph[row - 1][col].processed:
         neighbors.append(graph[r-1][c])
    if row < shape[0] - 1 and not graph[row + 1][c].processed:
            neighbors.append(graph[row + 1][c])
    if col > 0 and not graph[row][col-1].processed:
        neighbors.append(graph[row][c-1])
    if col < shape[1] - 1 and not graph[row][col+1].processed:
            neighbors.append(graph[row][col+1])
    return neighbors

def bubble_up(queue, ind):
    if ind <= 0:
        return queue
    pInd = (ind - 1) // 2
    if queue[ind].distFromSrc < queue[pInd].distFromSrc:
            queue[ind], queue[pInd] = queue[pInd], queue[ind]
            queue[ind].index_in_queue = ind
            queue[pInd].index_in_queue = pInd
            queue = bubble_up(queue, pInd)
    return queue
    
def bubble_down(queue, index):

    length=len(queue)
    lcInd = 2 * ind + 1
    rcInd = lcInd+1
    if lcInd >= length:
        return queue
    if lcInd < length and rcInd >= length: #just left child
        if queue[ind].distFromSrc > queue[lcInd].distFromSrc:
            queue[ind], queue[lcInd] = queue [lcInd], queue[ind]
            queue[ind].index_in_queue=index
            queue[lcInd].index_in_queue = lcInd
            queue = bubble_down(queue, lcInd)
    else:
        small = lcInd
        if queue[lcInd].d > queue[rcInd].d:
            small = rcInd
        if queue[small].d < queue[ind].d:
            queue[ind],queue[small]=queue[small],queue[ind]
            queue[ind].index_in_queue=index
            queue[small].index_in_queue=small
            queue = bubble_down(queue, small)
    return queue

# Get distance using RGB
def get_distance(img,u,v):
    return 0.1 + (float(img[v][0])-float(img[u][0]))**2+(float(img[v][1])-float(img[u][1]))**2+(float(img[v][2])-float(img[u][2]))**2

def find_shortest_path(img,src,dst):
    prioQueue = [] #min-heap priority queue
    source_x=src[0]
    source_y=src[1]
    dest_x = dst[0]
    dest_y = dst[1]
    imagerows,imagecols=img.shape[0],img.shape[1]
    matrix = np.full((imagerows, imagecols), None) #access by matrix[row][col]
    for r in range(imagerows):
        for c in range(imagecols):
            matrix[r][c]=Vertex(c,r)
            matrix[r][c].index_in_queue=len(prioQueue)
            prioQueue.append(matrix[r][c])
    matrix[source_y][source_x].distFromSrc =  0
    prioQueue = bubble_up(prioQueue, matrix[source_y][source_x].index_in_queue)

    while len(prioQueue) > 0:
        vert = prioQueue[0]
        vert.processed=True
        prioQueue[0]=prioQueue[-1]
        prioQueue[0].index_in_queue=0
        prioQueue.pop()
        prioQueue=bubble_down(prioQueue,0)
        neighbors = get_neighbors(matrix,vert.y, vert.x)
        for i in neighbors:
            dist = get_distance(img,(vert.y, vert.x),(i.y, i.x))
            if vert.distFromSrc + dist < i.distFromSrc:
                i.distFromSrc = vert.distFromSrc + dist
                i.xParent = vert.x
                i.yParent = vert.y
                ind = i.index_in_queue
                prioQueue = bubble_down(prioQueue, ind)
                prioQueue = bubble_up(prioQueue, ind)
                                   
    path=[]
    iter_v = matrix[dest_y][dest_x]
    path.append((dest_x,dest_y))
    while(iter_v.y!=source_y or iter_v.x!=source_x):
        path.append((iter_v.x,iter_v.y))
        iter_v=matrix[iter_v.parent_y][iter_v.parent_x]

    path.append((source_x,source_y))
    return path


image = cv.imread('maze_test.pgm')
cv.convert_to_grayscale(image)

find_shortest_path(img,STARTNODE,ENDNODE)