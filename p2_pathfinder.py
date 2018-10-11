
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

    for box in mesh['boxes']:
        if source_point[0] >= box[0] and source_point[0] <= box[1] and source_point[1] >= box[2] and source_point[1] <= box[3]:
            source_box = box
            boxes[box] = source_point 
            
        if destination_point[0] >= box[0] and destination_point[0] <= box[1] and destination_point[1] >= box[2] and destination_point[1] <= box[3]:
            destination_box = box
            boxes[box] = destination_point
    
    if source_box == None or destination_box == None:
        print("No path!")
        return path, boxes.keys()
    queue = [source_box]
    prev = {}
    detail_points = {}
    
    print(str(source_box))
    print(str(destination_box))

    detail_points[source_box] = source_point
    detail_points[destination_box] = destination_point

    if destination_box == source_box:
        path.insert(0, (destination_point, source_point))
        return path, boxes.keys()

    while queue:
        current_node = queue.pop()
        if current_node == destination_box:
            point1 = destination_point
            point2 = detail_points[current_node]
            print(str(detail_points[current_node]))
            path.insert(0, (point1, point2))
            while prev[current_node] != source_box:
                point1 = detail_points[current_node]
                boxes[current_node] = point1
                current_node = prev[current_node]
                point2 = detail_points[current_node]
                path.insert(0, (point1, point2))
            point1 = detail_points[current_node]
            point2 = source_point
            path.insert(0, (point1, point2))
            boxes[current_node] = point1
            print(path)
            print("Source: " + str(source_point))
            print("Destination: " + str(destination_point))
        else:
            for adj_node in mesh['adj'][current_node]:
                if adj_node not in prev:
                    prev[adj_node] = current_node
                    detail_points[adj_node] = calculate_point(current_node, adj_node, detail_points)
                    boxes[adj_node] = detail_points[adj_node]
                    queue.append(adj_node)
    
    if not path:
        print("No path!")
    
    return path, boxes.keys()

def calculate_point(current_node, next_node, detail_points):    
    
    if current_node[0] <= next_node[0] and next_node[0] <= current_node[1]:
        if current_node[0] <= next_node[1] and next_node[1] <= current_node[0]:
            lower_x = next_node[0]
            upper_x = next_node[1]
        else:
            lower_x = next_node[0]
            upper_x = current_node[1]
    elif next_node[0] <= current_node[0] and current_node[0] <= next_node[1]:
        if next_node[0] <= current_node[1] and current_node[1] <= next_node[1]:
            lower_x = current_node[0]
            upper_x = current_node[1]
        else:
            lower_x = current_node[0]
            upper_x = next_node[1]
    
    if current_node[2] <= next_node[2] and next_node[2] <= current_node[3]:
        if current_node[2] <= next_node[3] and next_node[3] <= current_node[3]:
            lower_y = next_node[2]
            upper_y = next_node[3]
        else:
            lower_y = next_node[2]
            upper_y = current_node[3]
    elif next_node[2] <= current_node[2] and current_node[2] <= next_node[3]:
        if next_node[2] <= current_node[3] and current_node[3] <= next_node[3]:
            lower_y = current_node[2]
            upper_y = current_node[3]
        else:
            lower_y = current_node[2]
            upper_y = next_node[3]
    """
    if current_node[1] <= next_node[0] and current_node[1] < next_node[1]:
        point = (next_node[0], max(min(detail_points[current_node][1], upper_y), lower_y))
    elif current_node[0] >= next_node[1] and current_node[0] > next_node[0]:
        point = (next_node[1], max(min(detail_points[current_node][1], upper_y), lower_y))
    elif current_node[2] >= next_node[3] and current_node[2] > next_node[2]:
        point = (max(min(detail_points[current_node][0], upper_x), lower_x), next_node[3])
    elif current_node[3] <= next_node[2] and current_node[3] < next_node[3]:
        point = (max(min(detail_points[current_node][0], upper_x), lower_x), next_node[2])
    else: 
        point = (0,0)
    """
    point = (max(min(detail_points[current_node][0], upper_x), lower_x), max(min(detail_points[current_node][1], upper_y), lower_y))

    return point
