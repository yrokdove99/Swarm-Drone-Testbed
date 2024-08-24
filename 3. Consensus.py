from inits import *

#! 1. set number of node
NUMOF_NODE = 3

#! 2. set angles of distance measurement sensors
#! Must be first element is front(0 degree)
#! and continues to clock wise sequence(+90: right, -90: left)
SENSOR_ANGLES = [0, 90, -90] 

#! 3. set Avoid obstacle parammeters
enableAvoidObstacle = True #! Enable Avoid Obstacle
ACHIEVED_CONDITION_DISTANCE = 0.1 #! Tuned here
BLENDING_DISTANCE, BLENDING_SIGMA = 0.2, 0.3
BLENDING_STATE = {'emergency': {'d':0.1, 'sigma':0.7}, 'warning': {'d':0.15, 'sigma':0.6}, 'week_warning':{'d':0.2, 'sigma': 0.5},'detected':{'d':BLENDING_DISTANCE, 'sigma': BLENDING_SIGMA}}
OBST_DETECT_MARGIN = 10

#! 4. set Test parameters
RUNNING_TIME_LIMIT = 180

#! underline parameters dependent on upper parameters you set
robots_info, NODE_IDX_LIST =  init_(NUMOF_NODE, SENSOR_ANGLES)
#! Not goal_position
dummy_point = (-1, -1)
#! 'scenario_name', shape, goal_pos
task = ['Consensus', (None, dummy_point)]

#! Main function start
example_running(task, robots_info, NODE_IDX_LIST, SENSOR_ANGLES, ACHIEVED_CONDITION_DISTANCE, BLENDING_STATE, OBST_DETECT_MARGIN, RUNNING_TIME_LIMIT, enableAvoidObstacle, ROTATION = None)