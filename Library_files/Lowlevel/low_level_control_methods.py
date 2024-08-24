from math import*
import numpy as np
import numpy.typing as npt

class LowLevelControlMethods:
    def __init__(self,BLENDING_STATE):
        self.BLENDING_STATE = BLENDING_STATE
        self.np_blending = np.vectorize(self.blending_function)#!
        self.calc_weight_option = {'avg':np.average, 'closest':np.max,}

    @staticmethod
    def get_obst_location(w_fw):
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
    
    @staticmethod
    def calc_limit_val_in_bound(val, ma, mi):
        min_bound = np.where(abs(val) < mi, 0, val)
        min_max_bound = np.where(abs(min_bound) > ma, ma, min_bound)
        min_max_bound = np.round(min_max_bound)
        return min_max_bound
    
    @staticmethod
    def get_PID_weight(err, err_max, desired_value_max):
        alpha = (desired_value_max)/err_max#1000 / 3
        weight = min(desired_value_max, alpha*round(err,2)) # 1000 -> 0.5 * 1000(diagonal), 0.25 * 1000 (forward)
        return weight
    
    @staticmethod
    def calc_desired_rotation_angle(heading, vxg, vyg):
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

    @staticmethod
    def get_achieve_state(state, desired_state, d, desired_d):
        if state == desired_state and d < desired_d:
            is_task_achieved = True 
        else:
            is_task_achieved = False
        return is_task_achieved
    
    #* dependency 
    def blending_function(self, distances:npt.NDArray[np.float64], opt='avg'):
        st = self.BLENDING_STATE
        do = distances

        if do < st['emergency']['d']: weight = st['emergency']['sigma']

        elif st['emergency']['d'] <= do < st['warning']['d'] : weight = st['warning']['sigma']

        elif st['warning']['d'] <= do < st['detected']['d'] : weight = st['detected']['sigma']

        else: weight = 0 # ordinary == safe state

        return weight
    
    def blending_function_np(self, arr):
        return self.np_blending(arr)


    def get_aggregated_weight(self, option, w_fw):
        return self.calc_weight_option[option](w_fw)
