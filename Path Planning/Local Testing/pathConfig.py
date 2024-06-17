import numpy as np

def origin():

        # Define origin for local coordinates in WGS84 (GPS)
        #                   LAT         LON
        origin = np.array([32.881110, -117.235472])

        return origin



def pickup():

        #Define pickup and drop off in local coordinates (m)
        #                  X  Y
        pickup = np.array([5, 15])

        return pickup



def dropoff(pathtype):

        # Define the dropoff locations in local coordinates (m)
        #         X   Y
        dropoff = [];
        if pathtype == 'Stars':
                dropoff = np.array([
                        [ 3,  9],
                        [18, 18],
                        [18,  3],
                        [ 5,  4],
                        [ 2, 19],
                        [ 8,  2],
                        [10, 18]
                ])
        elif pathtype == 'Circles':
                dropoff = np.array([
                        [ 3,  9],
                        [18, 18],
                        [18,  3],
                        [ 5,  4],
                        [ 2, 19],
                        [ 8,  2],
                        [10, 18]
                ])

        else:
                dropoff = np.array([
                        [ 3,  9],
                        [18, 18],
                        [18,  3],
                        [ 5,  4],
                        [ 2, 19],
                        [ 8,  2],
                        [10, 18]
                ])


        return dropoff



def obstacle():

        #Define center of each obstacle in local coordinates and the width and height of each obstacle (m)
        #         X   Y    W    H
        obstacle = np.array([
                [ 3,  4, 1.5, 1.5],
                [ 3, 12, 2.5,   2],
                [ 4,  1,   3, 1.5],
                [10,  2,   2, 1.5],
                [14, 16,   4,   4],
                [ 5, 18,   4,   1],
                [17,  9,   4,   5]
        ])

        return obstacle



def offset():

        # Define the desired offset from obstacles vertices (m)
        offset = 0.25

        return offset



def cardyn():

        # Define the associated dynamics of your car
        cardyn = {
                'min_turn_r': 0.75,
                'vel_avg': 1,
                'dx': 0.2
        }

        return cardyn