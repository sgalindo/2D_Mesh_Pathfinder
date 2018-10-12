
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
    
    print('Source box:', str(source_box))
    print('Destination box:', str(destination_box))

    # Special case where destination box is the same as the source box, return path
    if destination_box == source_box:
        path.insert(0, (destination_point, source_point))
        return path, boxes.keys()

    # Priority queue
    queue = [(0, source_box, 'forward'), (0, destination_box, 'back')]
    # Previous nodes (forward and back)
    prev_forward = {}
    prev_back = {}
    # Distances (forward and back)
    dist_forward = {source_box: 0}
    dist_back = {destination_box: 0}
    # Detail points (forward and back)
    detail_points_forward = {}
    detail_points_back = {}

    #Saving the source box and destination_point to their respective detail dicts
    detail_points_forward[source_box] = source_point
    detail_points_back[destination_box] = destination_point

    while queue:
        # Pop distance, node, and direction from queue
        current_dist, current_node, current_direction = heappop(queue)

        # If this node is in the opposite direction's prev[] list, we have reached the midpoint
        if (current_direction == 'forward' and current_node in prev_back) or (current_direction == 'back' and current_node in prev_forward):
            
            # Case when the source and destination boxes are adjacent
            if prev_forward[current_node] == source_box:
                path.insert(0, (source_point, detail_points_forward[current_node]))
                path.insert(0, (detail_points_forward[current_node], destination_point))
                return path, boxes.keys()
            
            # Get both detail points for the middle box 
            # (only this box has 2 since two line segments meet on either side)
            # Insert line segment to path
            point1 = detail_points_forward[current_node]
            point2 = detail_points_back[current_node]
            path.insert(0, (point1, point2))

            # Keep a copy of middle node for back direction
            current_node2 = current_node

            # Insert line segments from the forward direction until source is reached
            while prev_forward[current_node] != source_box:
                point1 = detail_points_forward[current_node]
                boxes[current_node] = point1
                current_node = prev_forward[current_node]
                point2 = detail_points_forward[current_node]
                path.insert(0, (point1, point2))
            
            # Insert source
            point1 = detail_points_forward[current_node]
            point2 = source_point
            path.insert(0, (point1, point2))
            boxes[current_node] = point1

            # Insert line segments from the back direction until source is reached
            while prev_back[current_node2] != destination_box:
                point1 = detail_points_back[current_node2]
                boxes[current_node2] = point1
                current_node2 = prev_back[current_node2]
                point2 = detail_points_back[current_node2]
                path.insert(0, (point1, point2))
            
            # Insert destination
            point1 = detail_points_back[current_node2]
            point2 = destination_point
            path.insert(0, (point1, point2))
            boxes[current_node2] = point1

            # Return path and explored boxes
            return path, boxes.keys()

        # For all nodes adjacent to current_node
        for adj_node in mesh['adj'][current_node]: 
            
            if current_direction == 'forward':
                # Calculate next point in adjacent box
                new_point = calculate_point(current_node, adj_node, detail_points_forward)
                # Add distance from current point to next to the total distance from start
                pathcost = current_dist + sqrt((new_point[0] - detail_points_forward[current_node][0])**2 + (new_point[1] - detail_points_forward[current_node][1])**2)
                # If adj_node has not been visited / has a better cost than the others
                if adj_node not in dist_forward or pathcost < dist_forward[adj_node]:
                    dist_forward[adj_node] = pathcost
                    prev_forward[adj_node] = current_node
                    detail_points_forward[adj_node] = new_point
                    pathcost += sqrt((new_point[0] - destination_point[0])**2 + (new_point[1] - destination_point[1])**2)
                    # Push pathcost, node, and its direction to heap
                    heappush(queue, (pathcost, adj_node, 'forward'))
            if current_direction == 'back':
                new_point = calculate_point(current_node, adj_node, detail_points_back)
                pathcost = current_dist + sqrt((new_point[0] - detail_points_back[current_node][0])**2 + (new_point[1] - detail_points_back[current_node][1])**2)
                if adj_node not in dist_back or pathcost < dist_back[adj_node]:
                    dist_back[adj_node] = pathcost
                    prev_back[adj_node] = current_node
                    detail_points_back[adj_node] = new_point
                    pathcost += sqrt((new_point[0] - source_point[0])**2 + (new_point[1] - source_point[1])**2)
                    heappush(queue, (pathcost, adj_node, 'back'))

    # If no path exists, print this
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
