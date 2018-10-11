
from math import inf, sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh
    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to
    Returns:
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    path = []
    boxes = {}
    source_box = None
    destination_box = None

    #Checks all nodes in 'boxes' to try and find the source and destination box
    for box in mesh['boxes']:
        if source_point[0] >= box[0] and source_point[0] <= box[1] and source_point[1] >= box[2] and source_point[1] <= box[3]:
            source_box = box
            boxes[box] = source_point 
            
        if destination_point[0] >= box[0] and destination_point[0] <= box[1] and destination_point[1] >= box[2] and destination_point[1] <= box[3]:
            destination_box = box
            boxes[box] = destination_point
    #If there is no source box or destination box, then it returns "No path!"
    if source_box == None or destination_box == None:
        print("No path!")
        return path, boxes.keys()

    #queue starts with the source_box and will be used to process the boxes for the path
    #prev stores the prev node for the node saved in the dict
    #detail_points is a dictionary of map points (x,y)

    queue = [source_box]
    prev = {}
    detail_points = {}
    dist = {source_box: 0}
    
    print('Source box:', str(source_box))
    print('Destination box:', str(destination_box))

    #Saving the source box and destination_point to the detail_points dict
    detail_points[source_box] = source_point
    detail_points[destination_box] = destination_point

    #Special case where destination box is the same as the source box, return path

    if destination_box == source_box:
        path.insert(0, (destination_point, source_point))
        return path, boxes.keys()

    """
    #Whenever you use path.insert, the format is (position, tuple)
    #The tuple is made up of two coordinates to make (x,y)
    while queue:
        #Pop off the node in the queue and save to current node
        current_node = queue.pop()
        #If the current node has reached the destination box, save the points to the path 
        if current_node == destination_box:
            #point1 and point2 are used to define the points of the box that they are in
            #****Come back and maybe flip point1 and point2****
            point1 = destination_point
            point2 = detail_points[current_node]
            print('detail_points dict[current_node]', str(detail_points[current_node]))
            path.insert(0, (point1, point2))
            #Goes down the prev dict until you reach the source box
            while prev[current_node] != source_box:
                point1 = detail_points[current_node]
                boxes[current_node] = point1
                current_node = prev[current_node]
                point2 = detail_points[current_node]
                print('detail_points dict[current_node]', str(detail_points[current_node]))
                path.insert(0, (point1, point2))
            #****Come back and maybe flip point1 and point2****
            point1 = detail_points[current_node]
            point2 = source_point
            path.insert(0, (point1, point2))
            boxes[current_node] = point1
            #print(path)
            print("Source: " + str(source_point))
            print("Destination: " + str(destination_point))

        #If we have not reached the destination box, then we continue to search for the path
        else:
            for adj_node in mesh['adj'][current_node]:
                #checks if the adj_node has already been discovered
                if adj_node not in prev:
                    prev[adj_node] = current_node
                    #calculating the point for the detail_points dict
                    detail_points[adj_node] = calculate_point(current_node, adj_node, detail_points)
                    boxes[adj_node] = detail_points[adj_node]
                    queue.append(adj_node)
    """

    while queue:
        current_node = heappop(queue)
        current_dist = dist[current_node]
        print(str(current_dist))

        # CHANGE LATER
        if current_node == destination_box:
            #point1 and point2 are used to define the points of the box that they are in
            point1 = destination_point
            point2 = detail_points[current_node]
            print('detail_points dict[current_node]', str(detail_points[current_node]))
            path.insert(0, (point1, point2))
            #Goes down the prev dict until you reach the source box
            while prev[current_node] != source_box:
                point1 = detail_points[current_node]
                boxes[current_node] = point1
                current_node = prev[current_node]
                point2 = detail_points[current_node]
                print('detail_points dict[current_node]', str(detail_points[current_node]))
                path.insert(0, (point1, point2))
            point1 = detail_points[current_node]
            point2 = source_point
            path.insert(0, (point1, point2))
            boxes[current_node] = point1
            #print(path)
            print("Source: " + str(source_point))
            print("Destination: " + str(destination_point))

        for adj_node in mesh['adj'][current_node]:
            detail_points[adj_node] = calculate_point(current_node, adj_node, detail_points)
            pathcost = current_dist + sqrt((detail_points[adj_node][0] - detail_points[current_node][0])**2 + (detail_points[adj_node][1] - detail_points[current_node][1])**2) 
            if adj_node not in dist or pathcost < dist[adj_node]:
                dist[adj_node] = pathcost
                prev[adj_node] = current_node
                heappush(queue, (pathcost, adj_node))

    if not path:
        print("No path!")
    
    return path, boxes.keys()

#Used to calculate the edge points for the path
def calculate_point(current_node, next_node, detail_points):    
    
    #Format of (x1,x2,y1,y2)
    #(x1,y1) is the upper left corner of the node
    #(x2,y2) is the upper left corner of the node
    #x1: small x coordinate
    #x2: big x coordinate
    #y1: small y coordinate
    #y2: big y coordinate
    #Check for the bounds of the x coordinate points
    #If the current node's small x is less than the next_node's small x and 
    #next node's small x is less than current node's big x, then set the 
    #lower x to next node's lower and upper x  to next node's upper
    if current_node[0] <= next_node[0] and next_node[0] <= current_node[1]:
        #Checks if current node's small x is bigger than next node's big x and
        #next node's big x is smaller than current node's small x
        #Means that lower and upper x bounds follows next nodes small and big x 
        #*******CHANGE HERE********** 
        if current_node[0] <= next_node[1] and next_node[1] <= current_node[1]:
            lower_x = next_node[0]
            upper_x = next_node[1]
        #Else, lower bound belongs to next node's x and upper bound belongs to current node's x 
        else:
            lower_x = next_node[0]
            upper_x = current_node[1]
    #Checks if next node's small x is bigger than current node's small x and
    #current node's small x is smaller than next node's big x
    elif next_node[0] <= current_node[0] and current_node[0] <= next_node[1]:
        #Checks if next node's small x is bigger than current node's big x and
        #current node's big x is smaller than next node's small x
        #Means that lower and upper x bounds follows current nodes small and big x 
        if next_node[0] <= current_node[1] and current_node[1] <= next_node[1]:
            lower_x = current_node[0]
            upper_x = current_node[1]
        #Else, lower bound belongs to current node's x and upper bound belongs to next node's x 
        else:
            lower_x = current_node[0]
            upper_x = next_node[1]
    
    #Check for the bounds of the y coordinate points
    #If the current node's small y is less than the next_node's small y and 
    #next node's small y is less than current node's big y, then set the 
    #lower y to next node's lower and upper y  to next node's upper
    if current_node[2] <= next_node[2] and next_node[2] <= current_node[3]:
        #Checks if current node's small y is bigger than next node's big y and
        #next node's big y is smaller than current node's small y
        #Means that lower and upper y bounds follows next nodes small and big y
        #*******CHANGE HERE********** 
        if current_node[2] <= next_node[3] and next_node[3] <= current_node[3]:
            lower_y = next_node[2]
            upper_y = next_node[3]
        #Else, lower bound belongs to next node's y and upper bound belongs to current node's y
        else:
            lower_y = next_node[2]
            upper_y = current_node[3]
    #Checks if next node's small y is bigger than current node's small y and
    #current node's small y is smaller than next node's big y
    elif next_node[2] <= current_node[2] and current_node[2] <= next_node[3]:
        #Checks if next node's small y is bigger than current node's big y and
        #current node's big y is smaller than next node's small y
        #Means that lower and upper y bounds follows current nodes small and big y 
        if next_node[2] <= current_node[3] and current_node[3] <= next_node[3]:
            lower_y = current_node[2]
            upper_y = current_node[3]
        #Else, lower bound belongs to current node's y and upper bound belongs to next node's y 
        else:
            lower_y = current_node[2]
            upper_y = next_node[3]
    point = (max(min(detail_points[current_node][0], upper_x), lower_x), max(min(detail_points[current_node][1], upper_y), lower_y))

    return point
