import os

ROBOT_DISTANCE_ANGLES = list(range(0, 360, 45))
ROBOT_MAX_SENSOR_DISTANCE = 100
ROBOT_DEFAULT_START_POS = (20, 560)

OBJECTIVE_DEFAULT_START_POS = (500, 50)

SIMBOTMAP_SIZE = (700, 600)
SIMBOTMAP_BOUNDING_LINES = (
    ((0, 0), (SIMBOTMAP_SIZE[0], 0)),
    ((SIMBOTMAP_SIZE[0], 0), (SIMBOTMAP_SIZE[0], SIMBOTMAP_SIZE[1])),
    ((SIMBOTMAP_SIZE[0], SIMBOTMAP_SIZE[1]), (0, SIMBOTMAP_SIZE[1])),
    ((0, SIMBOTMAP_SIZE[1]), (0, 0)),
)