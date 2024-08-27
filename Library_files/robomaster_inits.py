from typing import Dict
import time

from .robomaster_pkg import robot
from .robomaster_pkg import conn
from .robomaster_pkg.conn import config

from .robomaster_pkg import logger as mylog
import logging
mylog.setLevel(logging.DEBUG)

import sys
from datetime import datetime as dt
g_debug_msg = []
sys._getframe(0).f_code.co_name


def get_online_robot_info(timeout) -> Dict:
    ip ,sn = conn.scan_robot_ip_list(timeout=timeout)
    online = []
    for ip, sn in zip(ip, sn):
        if '159CJ' in sn:
            model = 's1'
        else:
            model = 'ep'
        online.append((ip, sn, model))
    online.sort()

    online_robots = {}
    for idx, (ip, sn, model) in enumerate(online):
        online_robots[idx] = (idx, ip, sn, model)

    return online_robots

import psutil
def check_overlap_bind_port():
    flag_port_free = 0
    while not flag_port_free:
        flag_port_free = 1
        datas = psutil.net_connections()
        for row in datas:
            if f"laddr=addr(ip='0.0.0.0', port={config.ROBOT_BROADCAST_PORT})" in str(row):
                flag_port_free = 0
                time.sleep(1)
                break
            else: continue

def get_robots(NUMOF_ROBOTS, proto_type):
    robots_list = []
    ip_list, sn_list = conn.scan_robot_ip_list(5)
    if NUMOF_ROBOTS != len(ip_list):
        raise Exception('Number of intended working robots and number of searched robots are different.\
        Check the NUMOF_ROBOTS value.')

    for node_index in range(NUMOF_ROBOTS):
        ep_robot = robot.Robot()
        print(f'[NODE:{node_index}] init start', end=' ')
        check_overlap_bind_port()
        ep_robot.initialize(conn_type='sta', sn=sn_list[node_index], proto_type='tcp') 
        robots_list.append(ep_robot)
        print(f'[NODE:{node_index}] init success')

    print('[INFO] robots are initialized')
    return robots_list