from numpy import pi
import numpy as np

# pillars = False
# stopQuadrantsCount = 12
# headless = True

# speed = -0.065
# # speed = -0.10

redMin = (0, 100, 60)
redMax = (5, 255, 255)
greenMin = (50, 50, 10)
greenMax = (80, 255, 255)

grayThresh = 65

crop_hight = 260

# centerstripheight = 80
centerheight = 480 // 2

minContourSize = 4.0
# contourSizeConstant = 1

axle_lenght = 0.25
tire_dia = 0.0425 * 2

# mtx = np.array([[601.79696678,   0.0,         329.41898336],
#  [  0.0,         617.44002116, 142.83666391],
#  [  0.0,           0.0,           1.0        ]])

# dist = np.array([[-6.83666733e-01, -4.95120581e+00,  1.62229680e-01, -1.92934685e-02, 2.22295767e+01]])


mtx = np.array([[495.44408865,   0.0,         294.45510998],
 [  0.0,         504.36905411, 259.77204181],
 [  0.0,           0.0,           1.0        ]])
dist = np.array([[-0.64066312,  0.41901729, -0.00366722,  0.01957175, -0.14404429]])
