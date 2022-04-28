import numpy as np
import matplotlib.pyplot as plt
import logging
from scipy.spatial import ConvexHull

logging.basicConfig(level=logging.INFO)


def generate_points():
    height_resolution = 15
    diameter_pts = 10
    cylinder_height = 100
    cylinder_diameter = 10

    cone_height = 50

    com_ratio = 0.5

    height = cylinder_height + cone_height
    com_height = height * com_ratio

    points = np.zeros((1, 3))

    logging.info(points)
    heights = np.arange(-com_height, height-com_height + height_resolution, height_resolution)

    logging.info(heights)

    for i in heights:
        if i > cylinder_diameter - com_height:
            norm_height =  i - (cylinder_height-com_height)
            diameter = cylinder_diameter*np.sqrt(cone_height*(cone_height-norm_height))/cone_height
        else:
            diameter = cylinder_diameter

        # diameter =  cylinder_diameter - cylinder_diameter/cone_height * (i-(cylinder_height-com_height)) if i > cylinder_height - com_height else cylinder_diameter
        for j in range(diameter_pts):
            new_point = np.zeros((1, 3))
            new_point[0] = [diameter * np.cos(j*2*np.pi/diameter_pts), diameter * np.sin(j*2*np.pi/diameter_pts), i]
            points = np.append(points, new_point, 0)
    
    return(points)

def convex_hull(points):
    hull = ConvexHull(points)
    for s in hull.simplices:
        s = np.append(s, s[0])

    return hull.simplices


points = generate_points()
hull_simplices = convex_hull(points)

logging.info(points)

fig1 = plt.figure()
ax = fig1.add_subplot(projection='3d')
for i in hull_simplices:
    i = np.append(i, i[0])
    ax.plot(points[i, 0], points[i, 1], points[i, 2], 'r-')

AXIS_LIMIT = 80
ax.set_xlim([-AXIS_LIMIT, AXIS_LIMIT])
ax.set_ylim([-AXIS_LIMIT, AXIS_LIMIT])
ax.set_zlim([-AXIS_LIMIT, AXIS_LIMIT])
plt.show()