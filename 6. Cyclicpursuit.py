from inits import *

#! 1. set number of node
NUMOF_NODE = 4

#! 2. set angles of distance measurement sensors
#! Must be first element is front(0 degree)
#! and continues to clock wise sequence(+90: right, -90: left)
SENSOR_ANGLES = [0, 90, -90] 

#! 3. set Avoid obstacle parameters
enableAvoidObstacle = True #! Enable Avoid Obstacle
ACHIEVED_CONDITION_DISTANCE = 0.1 #! Tuned here
BLENDING_DISTANCE, BLENDING_SIGMA = 0.2, 0.3
BLENDING_STATE = {'emergency': {'d':0.1, 'sigma':0.7}, 'warning': {'d':0.15, 'sigma':0.6}, 'week_warning':{'d':0.2, 'sigma': 0.5},'detected':{'d':BLENDING_DISTANCE, 'sigma': BLENDING_SIGMA}}
OBST_DETECT_MARGIN = 10

#! 4. set Test parameters
RUNNING_TIME_LIMIT = 180

#! underline parameters dependent on upper parameters you set
robots_info, NODE_IDX_LIST =  init_(NUMOF_NODE, SENSOR_ANGLES)
#! Set goal position 
DST = (0, 0)
#! Set desired distance
d = desired_distance = 1.71
d_cen = 1
#! Set formation shape
triangle_center = np.array(
[[0, d, d, d_cen],
[d, 0, d, d_cen],
[d, d, 0, d_cen],
[d_cen, d_cen, d_cen, 0]]
)
SHAPE = triangle_center

#! Set desired rotation angle
THETA = np.pi/6
ROTATION = np.array([
    [np.cos(THETA), -np.sin(THETA)],
    [np.sin(THETA), np.cos(THETA)]
])

#! 'scenario_name', shape, goal_pos
task = ['CyclicPursuit', (SHAPE, DST)]

#! Main function start
example_running(task, robots_info, NODE_IDX_LIST, SENSOR_ANGLES, ACHIEVED_CONDITION_DISTANCE, BLENDING_STATE, OBST_DETECT_MARGIN, RUNNING_TIME_LIMIT, enableAvoidObstacle, ROTATION)