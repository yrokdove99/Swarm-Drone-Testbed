from math import*
import numpy as np
import numpy.typing as npt
from numpy import exp as nexp 
from .normalize import *

class LowLevelControlMethods():
    def __init__(self, ESTM, BLENDING_STATE):
        self.ESTM = ESTM
        self.BLENDING_STATE = BLENDING_STATE
        self.calc_weight_option = {'avg':np.average, 'closest':np.max,}
        self.np_blending = None
        self.declare_np_blending()
        
    def calc_limit_val_in_bound(self, val, ma, mi):
        min_bound = np.where(abs(val) < mi, 0, val)
        min_max_bound = np.where(abs(min_bound) > ma, ma, min_bound)
        return min_max_bound
        
    def get_PID_weight(self, err, err_max, value_max):
        alpha = (value_max)/err_max#1000 / 3
        weight = min(value_max, alpha*round(err,2))
        return weight

    def calc_head_rotation_angle(self, heading, vxg, vyg):
        theta = heading
        deg_c = round(degrees(theta), 0) # heading of robot chassis
        deg_g = round(degrees(atan2(vyg, vxg)), 0) # goal position angle

        if deg_c < 0:
            if 0 < deg_g < deg_c + 180:
                if deg_g < 0: # case: b1
                    deg_h = +abs(deg_c-deg_g)
                else: # case: a1
                    deg_h = +abs(deg_c) + abs(deg_g)
                
            else:
                if deg_g < 0: # case: b2
                    deg_h = -(abs(deg_g) - abs(deg_c))
                else: # case: a2
                    deg_h = -(180 - abs(deg_c)) - (180 - abs(deg_g))

        else:
            if 0 <= deg_g <= deg_c: # case: c1
                deg_h = -abs(deg_c-deg_g)
            elif deg_c - 180 < deg_g < 0: # case: d1
                deg_h = -abs(deg_c)-abs(deg_g)

            elif deg_c <= deg_g: # case c2
                deg_h = abs(deg_c-deg_g)
            elif deg_g <= deg_c - 180 : # case d2
                deg_h = +(180 - deg_c) + (180 - abs(deg_g))

        return deg_h

    def blending_function(self, distances:npt.NDArray[np.float64], opt='avg'):
        st = self.BLENDING_STATE
        do = distances

        if do < st['emergency']['d']: weight = st['emergency']['sigma']

        elif st['emergency']['d'] <= do < st['warning']['d'] : weight = st['warning']['sigma']

        elif st['warning']['d'] <= do < st['detected']['d'] : weight = st['detected']['sigma']

        else: weight = 0 # ordinary == safe state

        return weight

    def declare_np_blending(self):
        self.np_blending = np.vectorize(self.blending_function)

    def blending_function_np(self, arr):
        return self.np_blending(arr)
        
    def get_obst_location(self, w_fw):
            w_fw_front =  w_fw[0]
            if w_fw_front != 0:
                obst_front = True
            else:
                obst_front = False

            w_fw_side = w_fw[1:]

            if np.sum(w_fw_side) != 0:
                obst_side = True
            else:
                obst_side = False

            return obst_front, obst_side

    def get_Ufw_Utask(self, rob, dist, task_vector, nonfunc=False):
        ESTM = self.ESTM
        w_fw = self.blending_function_np(np.append(0,dist))
        w_fw = w_fw[1:]

        obst_front, obst_side = self.get_obst_location(w_fw)
        u_task = np.array(task_vector).reshape(2,1) # heading(=front) vector
        if nonfunc:
            return 0, u_task, 0
        else:

            if obst_front or obst_side: # if Somewhere Obstacle detected

                rob.state = rob.state_undesired
                rob.obst_cnt += 1
                rob.obst_maintain_cnt = rob.OBST_DETECT_MARGIN

                if 3 < rob.obst_cnt:
                    v_fw_comp = ESTM.get_node_follow_wall_vector_comp(rob.node_idx) #(1,3)
                    epsilon = 10
                    v_fw_comp = v_fw_comp * nexp(1j*radians(epsilon)) # 90 +10 degree
                    real, imag = ESTM.decompose_real_imag(v_fw_comp) # real:(3,1), imag:(3,1)
                    real, imag = real.reshape(1,3), imag.reshape(1,3)
                    
                    m_fw = np.concatenate((real,imag), axis=0) # (2,3)
                    u_fw = m_fw.sum(axis=1).reshape(2,1)
                else:
                    m_fw = ESTM.get_node_follow_wall_vector_matrix(rob.node_idx) #(2,3)
                    u_fw = m_fw.sum(axis=1).reshape(2,1)

                u_fw = normalized_vector(u_fw)
                u_task = normalized_vector(u_task)
                
            else: # if No Obstacle detected
                rob.state = rob.state_desired
                rob.obst_cnt  = 0
                u_fw = np.array((0,0)).reshape(2,1)
                
            return u_fw, u_task, w_fw

    def get_aggregated_weight(self, option, w_fw):
        return self.calc_weight_option[option](w_fw)

    def synthesis_avoid_obstacle_vector(self, rob, dist, task_vector, option:str='closest', nonfunc=False):
        u_fw, u_task, w_fw = self.get_Ufw_Utask(rob, dist, task_vector, nonfunc)

        if nonfunc:
            return u_task
        else:
            w_ag = self.get_aggregated_weight(option, w_fw) #! weight_aggregated
            v_ao = w_ag*u_fw + (1-w_ag)*u_task
            
            return v_ao


    def get_achieve_state(self, state, desired_state, d, desired_d):
        if state == desired_state and d < desired_d:
            is_task_achieved = True 
        else:
            is_task_achieved = False
        return is_task_achieved