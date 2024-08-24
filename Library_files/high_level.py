from .add_path import * 

import time
from timeit import default_timer as timer

class HighLevelControl():
    def __init__(self, purpose=None, RUNNING_TIME=60, SLEEP_TIME=0.01, scenarios=None):
        self.purpose = purpose
        self.RUNNING_TIME_LIMIT = RUNNING_TIME
        self.HLC_THREAD_SLEEP_TIME = SLEEP_TIME
        self.scenarios = scenarios

    def _is_over_running_time(self, HLC_elapsed_time):
        """
        Le us know our code operation limit time over or not

        Return:
            over: True
            not: False
        """
        if (HLC_elapsed_time < self.RUNNING_TIME_LIMIT):
            return False
        else:
            return True

    def _return_calculated_purpose(self):
        """
        Do noting now(ver.2023-01-17)

        centralized moving algorithms goes here and return purpose
        """
        purpose = self.purpose
        return purpose

    def high_level_control_routine(self):
        ######################################################################

        #     #* give middle level task to middle level control part
            # g_purpose = self._return_calculated_purpose() # updating our purpose
        g_purpose = self.purpose
        if g_purpose:
            g_purpose = self._return_calculated_purpose() # updating our purpose
        else:
            g_purpose = None # no task anymore
        ######################################################################

        #* ----- adjust code running time -----
        time.sleep(self.HLC_THREAD_SLEEP_TIME)

        return g_purpose
        ##################################### high level control end #####################################