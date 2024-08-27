# from .add_debug import *
from .myrobomaster import MyRobomaster

import numpy as np
import numpy.linalg as np_la
from typing import Dict

import time
from timeit import default_timer as timer

class MiddleLevelTaskHandler(object):
    def __init__(self, estm, NUMOF_NODES, task_type: str, ACHIEVED_CONDITION_DISTANCE=None) -> None:
        self.ESTM = estm
        self.NUMOF_NODES = NUMOF_NODES
        self.handler_type = task_type
        self.criteria = ACHIEVED_CONDITION_DISTANCE # unit: millimeter

    def calc_task_to_vector(self, node_idx): # Method Overriding
        pass
    
class SetEachGoal(MiddleLevelTaskHandler):
    def __init__(self, estm, NUMOF_NODES, dst):
        super().__init__(estm, NUMOF_NODES, 'set_each_goal')
        self.goal_pos = dst[1] # destination

    # Calc task_vector to achieve go to goal problem
    def calc_task_to_vector_matrix(self):
        """
        Calculate vector from goal position
        """
        # x_position, y_position to goal
        ESTM = self.ESTM
        N = self.NUMOF_NODES
        m = ESTM.get_node_position_matrix() #(N,3)

        x_g, y_g = self.goal_pos
        x_i, y_i = m[:, 0], m[:, 1]#, m[:, 0]# x, y, theta -> (x, y)
        v_ix, v_iy = (x_g - x_i).reshape(1,N), (y_g - y_i).reshape(1,N)
        v_mat = np.concatenate((v_ix,v_iy),axis=0).reshape(N,2)
        return v_mat

class Consensus(MiddleLevelTaskHandler):
    def __init__(self, estm, NUMOF_NODES, dummy=None):
        super().__init__(estm, NUMOF_NODES, 'Consensus')
        self.ADJ_RADIUS = 0.7

    def get_adjacency_matrix(self):
        ESTM = self.ESTM
        N = self.NUMOF_NODES
        A = np.zeros((N, N))
        NORMALIZATION = 1000
        m = ESTM.get_node_position_matrix() #(N,3)
        p = m[:,:-1] #(x_i, y_i)
        for i in range(N):
            p_i = p[i,:]
            v_i = p - p_i

            x_i = v_i[:,0]
            y_i = v_i[:,1]
            d = np.sqrt(np.square(x_i)+np.square(y_i)) / NORMALIZATION
            a = np.where(d < self.ADJ_RADIUS, 1, 0)

            A[i,:] = a
        return A

    def calc_task_to_vector_matrix(self):
        """
        Calc task vector to achieve consensus problem and set task vector for Low level
        """
        ESTM = self.ESTM
        N = self.NUMOF_NODES
        m = ESTM.get_node_position_matrix() #(N,3)
        p = m[:,:-1] #(x_i, y_i)
        A = self.get_adjacency_matrix()
        vector_matrix = np.zeros((N, 2))
        for i in range(N):
            p_i = p[i,:]
            a_i = A[i,:]
            vectors_i = (p - p_i)*((1-a_i).reshape(N,1))
            vsum = np.sum(vectors_i, axis=0).reshape(1,2)
            vector_matrix[i,:] = vsum
        
        return vector_matrix

class Formation(MiddleLevelTaskHandler):
    def __init__(self, estm, NUMOF_NODES, desired_distance:np.array=None, ACHIEVED_CONDITION_DISTANCE=None):

        super().__init__(estm, NUMOF_NODES, 'Formation', ACHIEVED_CONDITION_DISTANCE)

        self.D = desired_distance[0]

    def get_desired_dist(self):
        return self.D

    def calc_task_to_vector_matrix(self):
        """
        Calc task vector to achieve formation problem and set task vector for Low level
        """
        ESTM = self.ESTM
        N = self.NUMOF_NODES
        m = ESTM.get_node_position_matrix()
        p = m[:,:-1] #(x_i, y_i)
        vector_matrix = np.zeros((N, 2))
        D = self.get_desired_dist()
        NORMALIZATION = 1000
        for i in range(N):
            d_i = D[i,:]
            p_i = p[i,:] # (n,2) x (n,1) -> (n,2)
            v = (p - p_i)
            norm_ = np_la.norm((p - p_i),axis=1) / NORMALIZATION
            diff_norm_d_i = ((norm_ - d_i).reshape(N,1))
            vectors_i = v * diff_norm_d_i # (N,2)
            vsum = np.sum(vectors_i, axis=0).reshape(1,2)
            criteria = self.criteria
            # if desired distance is not met and norm(vsum) is almost zero vector
            if np.any(criteria < np.abs(diff_norm_d_i * NORMALIZATION)) and np_la.norm(vsum) < criteria:
                vsum = np.array((criteria+criteria/10, criteria+criteria/10))

            vector_matrix[i,:] = vsum

        return vector_matrix

class LeaderFollowing(MiddleLevelTaskHandler):
    def __init__(self, estm, NUMOF_NODES, goal, ACHIEVED_CONDITION_DISTANCE):
        desired_distance, dst = goal
        super().__init__(estm, NUMOF_NODES, 'LeaderFollowing', ACHIEVED_CONDITION_DISTANCE)
        try:
            if desired_distance.any(): # exist value
                pass
        except:
            raise Exception('desired_distance is empty, must set to')

        self.leader_dst = dst # destination
        self.D = desired_distance #(N, N)
        self.default_leader_idx = 0 #! 
        self.formation_achieved = 0 # False
        self.NORMALIZATION = 1000

    def blending_leader_node_gtg_vector(self, node_idx, vsum):
        if node_idx == self.default_leader_idx:
            vector_leader = self.get_leader_goal_vector()
            weight = self.formation_achieved # 0 or 1
            vsum = vsum + weight*vector_leader
        
        return vsum

    def prevent_zero_task(self, diff_norm_d_i, vsum):
        criteria = self.criteria
        # if desired distance is not met and norm(vsum) is almost zero vector
        if np.any(criteria < np.abs(diff_norm_d_i*self.NORMALIZATION)) and np_la.norm(vsum) < criteria:
            vsum = np.array((criteria+10, criteria+10))
        else:
            pass

        return vsum

    def _exist_formation_achieved(self, vector_matrix):
        result = 0
        criteria = self.criteria
        formation_achieved_matrix = np_la.norm(vector_matrix, axis=1)
        formation_achieved_matrix = np.where(formation_achieved_matrix < criteria, 1, 0)

        if np.all(formation_achieved_matrix == 1):
            result = 1

        return result
    # Calc task_vector to achieve go to goal problem
    def get_leader_goal_vector(self):
        """
        Calculate vector from goal position
        """
        # x_position, y_position to goal
        ESTM = self.ESTM
        leader_idx = self.default_leader_idx
        m = ESTM.get_node_position_matrix()[:,:-1]#(N,3)

        goal_pos = np.array(self.leader_dst)
        leader_pos = m[leader_idx,:]
        vector_leader = goal_pos - leader_pos

        return vector_leader #! shape?

    def calc_shape_to_D():
        pass
    def get_desired_dist(self):
        return self.D

    def calc_task_to_vector_matrix(self):
        """
        Calc task vector to achieve leaderfollowing problem and set task vector for Low level
        """
        ESTM = self.ESTM
        N = self.NUMOF_NODES
        m = ESTM.get_node_position_matrix()
        p = m[:,:-1] #(x_i, y_i)
        vector_matrix = np.zeros((N, 2))
        D = self.get_desired_dist()
        
        for i in range(N):
            d_i = D[i,:]
            p_i = p[i,:] # (n,2) x (n,1) -> (n,2)
            v = (p - p_i)
            norm_ = np_la.norm((p - p_i),axis=1)/self.NORMALIZATION
            diff_norm_d_i = ((norm_ - d_i).reshape(N,1))
            vectors_i = v * diff_norm_d_i # (N,2)
            vsum = np.sum(vectors_i, axis=0).reshape(1,2) #!shape?

            #! this part is key-code of Leader do GTG
            vsum = self.blending_leader_node_gtg_vector(node_idx=i, vsum=vsum)

            vsum = self.prevent_zero_task(diff_norm_d_i, vsum)

            vector_matrix[i,:] = vsum

        formation_achieved_matrix = np_la.norm(vector_matrix, axis=1)
        formation_achieved_matrix = np.where(formation_achieved_matrix < self.criteria, 1, 0)

        if np.all(formation_achieved_matrix == 1):
            self.formation_achieved = 1

        # change leader vector to Go to goal vector
        return vector_matrix

class CyclicPursuit(MiddleLevelTaskHandler):
    def __init__(self, estm, NUMOF_NODES, goal, ACHIEVED_CONDITION_DISTANCE, ROTATION):
        desired_distance, dst = goal
        super().__init__(estm, NUMOF_NODES, 'CyclicPursuit', ACHIEVED_CONDITION_DISTANCE)

        self.leader_dst = dst # destination
        self.default_leader_idx = 3 #! [3,5,7,9]
        self.formation_achieved = 0 # False
        self.ROTATION = ROTATION
        self.D = desired_distance #(N, N)
        self.NORMALIZATION = 1000

    # Calc task_vector to achieve go to goal problem
    def get_leader_goal_vector(self):
        """
        Calculate vector from goal position
        """
        # x_position, y_position to goal
        ESTM = self.ESTM
        leader_idx = self.default_leader_idx
        m = ESTM.get_node_position_matrix()[:,:-1]#(N,3)

        goal_pos = np.array(self.leader_dst)
        leader_pos = m[leader_idx,:]
        vector_leader = goal_pos - leader_pos
        return vector_leader
    
    def get_desired_dist(self):
        return self.D
    
    def blending_leader_node_gtg_vector(self, node_idx, vsum):
        if node_idx == self.default_leader_idx:
            vector_leader = self.get_leader_goal_vector()
            weight = self.formation_achieved 
            vsum = vsum + weight*vector_leader

        return vsum
    
    def _exist_formation_achieved(self, vector_matrix):
        result = 0
        criteria = self.criteria
        formation_achieved_matrix = np_la.norm(vector_matrix, axis=1)
        formation_achieved_matrix = np.where(formation_achieved_matrix < criteria, 1, 0)

        if np.all(formation_achieved_matrix == 1):
            result = 1

        return result
    
    def calc_task_to_vector_matrix(self):
        """
        Calc task vector to achieve cyclic pursuit problem and set task vector for Low level
        """
        ESTM = self.ESTM
        N = self.NUMOF_NODES
        R = self.ROTATION
        m = ESTM.get_node_position_matrix()
        p = m[:,:-1] #(x_i, y_i)
        D = self.get_desired_dist()
        vector_matrix = np.zeros((N, 2))
        leader_idx = self.default_leader_idx
        weight = self.formation_achieved
        v = p[:leader_idx,]
        p_l = p[leader_idx,:]
        v = v - p_l
        rotated_vectors = (R @ v.T).T
        vector_matrix[:leader_idx,:] = weight*(rotated_vectors - v)

        for i in range(N):
            d_i = D[i,:]
            p_i = p[i,:] # (n,2) x (n,1) -> (n,2)
            v = (p - p_i)
            norm_ = np_la.norm((p - p_i),axis=1)/self.NORMALIZATION
            diff_norm_d_i = ((norm_ - d_i).reshape(N,1))
            vectors_i = v * diff_norm_d_i # (N,2)
            vsum = np.sum(vectors_i, axis=0).reshape(1,2)

            vsum = self.blending_leader_node_gtg_vector(node_idx=i, vsum=vsum)
            vector_matrix[i,:] = vsum + vector_matrix[i,:]

        formation_achieved_matrix = np_la.norm(vector_matrix, axis=1)
        formation_achieved_matrix = np.where(formation_achieved_matrix < self.criteria, 1, 0)

        if np.all(formation_achieved_matrix == 1):
            self.formation_achieved = 1
        else:
            pass

        return vector_matrix

# MyRobomaster = None
class MiddleLevelControl():
    def __init__(self, estm, robots:Dict[int, MyRobomaster], ACHIEVED_CONDITION_DISTANCE, ROTATION=None):
        self.estm = estm
        self.robots = robots
        self.NUMOF_NODES = len(robots.keys())
        self._middle_level_task = None
        self._latest_middle_level_task = None
        self.task_handler = None
        self.task_vector_matrix = None
        self.ACHIEVED_CONDITION_DISTANCE = ACHIEVED_CONDITION_DISTANCE * 1000 # meter to millimeter
        self.MLC_THREAD_SLEEP_TIME = 0.01
        self.ROTATION = ROTATION

    def set_middle_level_task(self, task):
        self._middle_level_task = task

    def _exist_middle_level_task(self):
        if self._middle_level_task: return True
        else: return False

    def _get_handler(self, middle_level_task):
        task_type, goal = middle_level_task

        if task_type == 'GoToGoal': handler = SetEachGoal(self.estm, self.NUMOF_NODES, goal)
        elif task_type == 'Consensus': handler = Consensus(self.estm, self.NUMOF_NODES, goal)
        elif task_type == 'Formation': handler = Formation(self.estm, self.NUMOF_NODES, goal, self.ACHIEVED_CONDITION_DISTANCE)
        elif task_type == 'LeaderFollowing': handler = LeaderFollowing(self.estm, self.NUMOF_NODES, goal, self.ACHIEVED_CONDITION_DISTANCE)
        elif task_type == 'CyclicPursuit': handler = CyclicPursuit(self.estm, self.NUMOF_NODES, goal, self.ACHIEVED_CONDITION_DISTANCE, self.ROTATION)
        else:
            handler = None
            raise Exception('handler is unknown type')

        return handler

    def update_low_level_task_vector(self, task_vector_matrix):
        for i in self.robots.keys():
            vx, vy = task_vector_matrix[i,:]
            self.robots[i].set_task_vector(vx, vy)

    def update_low_level_task_vector_matrix(self, task_vector_matrix):
        self.task_vector_matrix = task_vector_matrix

    def middle_level_control_routine(self, g_purpose=None):
        MLC_start_time = timer()
        #* ---------- update high level purpose ----------
        self.set_middle_level_task(g_purpose)

        if self._exist_middle_level_task():
            #* get handler for middle level task
            #* Update handler for middle level task if only task is changed
            if self._middle_level_task != self._latest_middle_level_task:
                self._latest_middle_level_task = self._middle_level_task
                self.task_handler = self._get_handler(self._middle_level_task)

            task_vector_matrix = self.task_handler.calc_task_to_vector_matrix()
        else:
            N = self.NUMOF_NODES
            task_vector_matrix = np.zeros((N, 2))

        #* ----- Adjust code running time -----
        MLC_elapsed_time = timer() - MLC_start_time
        time.sleep(self.MLC_THREAD_SLEEP_TIME - MLC_elapsed_time if MLC_elapsed_time < self.MLC_THREAD_SLEEP_TIME else 0)

        return task_vector_matrix
        ##################################### middle level control end #####################################