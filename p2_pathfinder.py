
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

    for box in mesh['boxes']:
        if source_point[0] >= box[0] and source_point[0] <= box[1] and source_point[1] >= box[2] and source_point[1] <= box[3]:
            source_box = box
            boxes[box] = source_point 
            
        if destination_point[0] >= box[0] and destination_point[0] <= box[1] and destination_point[1] >= box[2] and destination_point[1] <= box[3]:
            destination_box = box
            boxes[box] = destination_point
    
    queue = [source_box]
    prev = {}
    
    while queue:
        current_node = queue.pop()
        if current_node == destination_box:
            point2 = (int((current_node[0] + current_node[1]) / 2), int((current_node[2] + current_node[3]) / 2))
            while current_node != source_box:
                point1 = (int((current_node[0] + current_node[1]) / 2), int((current_node[2] + current_node[3]) / 2))
                boxes[current_node] = point1
                current_node = prev[current_node]
                point2 = (int((current_node[0] + current_node[1]) / 2), int((current_node[2] + current_node[3]) / 2))
                path.insert(0, (point1, point2))
            point1 = (int((current_node[0] + current_node[1]) / 2), int((current_node[2] + current_node[3]) / 2))
            path.insert(0, (point1, point2))
            boxes[current_node] = point1
        else:
            for adj_node in mesh['adj'][current_node]:
                if adj_node not in prev:
                    prev[adj_node] = current_node
                    queue.append(adj_node)
    
    print(boxes)
    print(path)
    return path, boxes.keys()
