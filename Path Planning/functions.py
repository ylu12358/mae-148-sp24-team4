import numpy as np


def plotEnvironment(pickup, dropoff, obstacle, local):
    # Import matplotlib
    import matplotlib.pyplot as plt
    
    # Establish variable for use in plotting
    p = np.vstack((pickup, dropoff))
    colors = np.random.rand(len(dropoff) + 1, 3)

    # Plot all avoidance objects
    plt.figure(1)
    for i in range(obstacle.shape[1]):
        plt.plot(obstacle[:, 0, i], obstacle[:, 1, i])

    # Plot the pickup and drop off locations
    for i in range(len(dropoff) + 1):
        if i == 0:
            plt.plot(p[i, 0], p[i, 1], '^', color=colors[i], linewidth=2)
        else:
            plt.plot(p[i, 0], p[i, 1], 'o', color=colors[i], linewidth=2)

    # Plot the path lines
    plt.plot(local[:, 0], local[:, 1], 'k--')
    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def paramInterp(local, cardyn):
    # Import cubic spline library
    from scipy.interpolate import CubicSpline

    # Create the cubic spline
    sp = CubicSpline(np.arange(local.shape[1]), local.T)

    # Define time steps
    tstep = cardyn.dx / cardyn.vel_avg
    t = np.arange(0, sp.x[-1] + tstep, tstep)

    # Evaluate the spline at the given time steps
    sp_sol = sp(t).T

    return sp_sol


def convertToWGS84(input_coords):
    # Import conversion library
    from pyproj import Transformer

    # Transpose to get local
    local = input_coords.T

    # Create a zero array for z
    z = np.zeros((local.shape[1],))

    # Define the local coordinate system and WGS84 coordinate system
    local_crs = 'epsg:32611'  # UTM Zone 11N
    wgs84_crs = 'epsg:4326'  # WGS84

    # Create a transformer object
    transformer = Transformer.from_crs(local_crs, wgs84_crs, always_xy=True)

    # Perform the transformation
    lat, lon = transformer.transform(local[0], local[1])
    alt = z  # Assuming altitude is zero for all points

    # Combine the results into GPS array
    GPS = np.vstack((lat, lon, alt)).T

    return GPS