import numpy as np
from scipy.spatial.distance import cdist


def checkLineRect(obstacle, path):
    # Unpack obstacle and path
    _, _, w, h = obstacle
    nw, ne, sw, se = convertXYWHtoCorners(*obstacle)
    corners = np.round(np.vstack([nw, ne, sw, se]), 8)
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
        # print("NO SIDE \n")
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
        # print("COLLINEAR \n")
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
        # print(left)
        # print(bottom)
        # print(right)
        # print(top)
        # print("obstacle")
        # print(obstacle)
        # print("path")
        # print(path)
        # print("intersections")
        # print(intersections)
        # print("CORNER ONLY \n")
        return False, None
    
    # print(left)
    # print(inter_l)
    # print(bottom)
    # print(inter_b)
    # print(right)
    # print(top)
    # print("obstacle")
    # print(obstacle)
    # print("path")
    # print(path)
    # print("intersections")
    # print(intersections)

    # Return early if path is parallel to side
    # If x or y values of intersections are equal
    if intersections[0, 0] == intersections[1, 0] or intersections[0, 1] == intersections[1, 1]:
        # Initialize closest corner distance
        closest = cdist([a1], [a2])

        # Loop through every corner
        for i in range(len(corners)):
            # Find distnce between corner and start point of path
            corner_dist = cdist([a1], [corners[i]])
            
            # If distance is less than previous closest corner distance, store distance and corner
            if corner_dist < closest:
                closest = corner_dist
                closest_corner = corners[i]
            
        # print("corner")
        # print(closest_corner)
        # print("                        ")

        return True, closest_corner

    # Find nearest corner
    perp_diff = 90
    for i in range(len(corners)):
        a = intersections[0]
        b = intersections[1]
        center = corners[i]
        a_dist = cdist([a], [center])
        b_dist = cdist([b], [center])

        # Ignore corner if any of the intersection points are directly on the corner
        if a_dist == 0 or b_dist == 0:
            continue

        # Calculate angle formed by intersection points
        angle = calculateAngle(center, a, b)

        # If angle is more perpendicular than previous most perpendicular angle
        if abs(90 - angle) < perp_diff:
            perp_diff = abs(90 - angle)
            nearest_corner = corners[i]
    corner = nearest_corner

    # print("corner")
    # print(corner)
    # print("                        ")

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
    uA = round( ( s2x * (a1y-b1y) - s2y * (a1x-b1x) ) / determinant, 8)
    uB = round( ( s1x * (a1y-b1y) - s1y * (a1x-b1x) ) / determinant, 8)

    # Return intersection
    if 0 <= uA <= 1 and 0 <= uB <= 1:
        ix = round(a1x + uA * s1x, 8)
        iy = round(a1y + uA * s1y, 8)
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
    rectangle[2] += 2*offset
    rectangle[3] += 2*offset
    return rectangle


def calculateAngle(center, p1, p2):
    # Convert points to vectors from the center
    v1 = np.array(p1) - np.array(center)
    v2 = np.array(p2) - np.array(center)

    # Calculate the dot product of the vectors
    dot_product = np.dot(v1, v2)
    
    # Calculate the magnitudes of the vectors
    magnitude1 = np.linalg.norm(v1)
    magnitude2 = np.linalg.norm(v2)

    # Calculate the cosine of the angle
    cos_theta = dot_product / (magnitude1 * magnitude2)

    # Ensure the cosine value is within the valid range for arccos [-1, 1]
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    # Calculate the angle and convert to degrees
    angle_deg = np.degrees(np.arccos(cos_theta))

    return angle_deg