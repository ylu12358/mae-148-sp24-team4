import numpy as np
from scipy.spatial.distance import cdist


def checkLineRect(obstacle, path):
    # Unpack obstacle and path
    _, _, w, h = obstacle
    nw, ne, sw, se = convertXYWHtoCorners(*obstacle)
    corners = np.vstack([nw, ne, sw, se])
    a1, a2 = path

    # Check for side collisions
    left, inter_l = checkLineLine(a1, a2, nw, sw)
    bottom, inter_b = checkLineLine(a1, a2, sw, se)
    right, inter_r = checkLineLine(a1, a2, se, ne)
    top, inter_t = checkLineLine(a1, a2, ne, nw)

    # Calculate number of side intersections
    n_inter = sum([left, bottom, right, top])

    # Return early if there are no side intersections
    if n_inter == 0:
        return False, None
    
    # Assemble corners into sides
    sides = [
        (nw, ne),  # Top side
        (sw, se),  # Bottom side
        (nw, sw),  # Left side
        (ne, se)   # Right side
    ]

    # Check if path is collinear to any of the sides, return early if so
    if any(checkLineCollinear(a1, a2, side[0], side[1]) for side in sides):
        return False, None

    # Assemble side collisions into pairs
    pairs = [
        (left, inter_l),
        (bottom, inter_b),
        (right, inter_r),
        (top, inter_t)
    ]
    
    # Only stack non-empty intersections
    filtered_tuples = [t for flag, t in pairs if flag]
    if filtered_tuples:
        inter_stacked = np.vstack(filtered_tuples)
        # Remove duplicates by converting to a set
        intersections = np.array(list(set(map(tuple, inter_stacked))))

    # Calculate how many intersection points lie on an obstacle corner
    n_intercorner = len(set(map(tuple, corners)) & set(map(tuple, intersections)))

    # Return early if the path is okay
    if n_inter == 2 and n_intercorner == 1:
        return False, None
        
    # Find nearest corner
    nearest_dist = w+1 if w > h else h+1
    for i in range(len(corners)):
        a = intersections[0]
        b = intersections[1]
        a_dist = cdist([a], [corners[i]])
        b_dist = cdist([b], [corners[i]])
        if a_dist == 0 or b_dist == 0:
            continue
        average_dist = (a_dist + b_dist)/2
        if average_dist < nearest_dist:
            nearest_dist = average_dist
            nearest_corner = corners[i]
    corner = nearest_corner

    # Return intersection
    return True, corner


def checkLineLine(a1, a2, b1, b2):
    # Unpack points
    a1x, a1y = a1
    a2x, a2y = a2
    b1x, b1y = b1
    b2x, b2y = b2

    # Calculate changes in x and y
    s1x = a2x - a1x
    s1y = a2y - a1y
    s2x = b2x - b1x
    s2y = b2y - b1y

    determinant = (b2y-b1y) * s1x - (b2x-b1x) * s1y

    # Check for parallel lines
    if determinant == 0:
        return False, None
    
    # Calculate normalized distance to intersection point
    uA = ( s2x * (a1y-b1y) - s2y * (a1x-b1x) ) / determinant
    uB = ( s1x * (a1y-b1y) - s1y * (a1x-b1x) ) / determinant

    # Return intersection
    if 0 <= uA <= 1 and 0 <= uB <= 1:
        ix = a1x + uA * s1x
        iy = a1y + uA * s1y
        intersection = np.array([ix, iy])
        return True, intersection
    
    return False, None


def checkLineCollinear(a1, a2, b1, b2):
    return check3PointsCollinear(a1, a2, b1) and check3PointsCollinear(a1, a2, b2)


def check3PointsCollinear(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    return (y2 - y1) * (x3 - x1) == (y3 - y1) * (x2 - x1)


def convertXYWHtoCorners(x, y, w, h):
    nw = np.array([x - w/2, y + h/2])
    ne = np.array([x + w/2, y + h/2])
    sw = np.array([x - w/2, y - h/2])
    se = np.array([x + w/2, y - h/2])
    return nw, ne, sw, se


def offsetRectangle(rectangle, offset):
    rectangle[2] += offset
    rectangle[3] += offset
    return rectangle