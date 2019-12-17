# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
A script to fly 5 Crazyflies in formation. One stays in the center and the
other four fly aound it in a circle. Mainly intended to be used with the
Flow deck.
The starting positions are vital and should be oriented like this

     >

^1    +2    v0

     <

The distance from the center to the perimeter of the circle is around 0.5 m

"""
import math
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
import pandas as pd

# f = open("circle-swarm-log3.csv", 'w')

drone1_pos = (0, 0, 0)
drone2_pos = (0, 0, 0)
drone1_stab = (0, 0, 0, 0)
drone2_stab = (0, 0, 0, 0)


"""
Run 1: d = 1.0 URI0=01 URI1=E7
Run 2: d = 1.0 URI0=E7 URI1=01
Run 2: d = 0.5 URI0=E7 URI1=01
"""

df = pd.DataFrame()
value_dict = {}

# Change uris according to your setup
URI0 = 'radio://0/80/2M/E7E7E7E7E7'
URI1 = 'radio://0/80/2M/E7E7E7E702'
# URI2 = 'radio://0/80/2M/E7E7E7E702'
# URI3 = 'radio://0/80/2M/E7E7E7E702'
# URI4 = 'radio://0/80/2M/E7E7E7E703'

# d: diameter of circle
# z: altituce
params0 = {'d': 1.0, 'z': 0.5, 'yaw':0.0}
params1 = {'d': 1.0, 'z': 0.5, 'yaw':60.0}
# params2 = {'d': 0.0, 'z': 0.5, 'yaw':0.0}
# params3 = {'d': 1.0, 'z': 0.3}
# params4 = {'d': 1.0, 'z': 0.3}


uris = {
    URI0,
    URI1,
    # URI2,
    # URI3,
    # URI4,
}

params = {
    URI0: [params0],
    URI1: [params1],
    # URI2: [params2],
    # URI3: [params3],
    # URI4: [params4],
}

def check_loc_dist(cf):
    global drone1_pos
    global drone2_pos
    
    # print(drone1_pos, drone2_pos)

    if (drone1_pos != (0,0,0) and drone2_pos != (0,0,0)):
        total_dis = math.sqrt((drone1_pos[0]-drone2_pos[0])**2 + (drone1_pos[1]-drone2_pos[1])**2 + (drone1_pos[2]-drone2_pos[2])**2)
        total_diff = 0.1 < total_dis < 1.8
        x_diff = 0 < abs(drone1_pos[0] - drone2_pos[0]) <= 1.5
        y_diff = 0 < abs(drone1_pos[1] - drone2_pos[1]) <= 1.8
        z_diff = 0 < abs(drone1_pos[2] - drone2_pos[2]) <= 0.7435006574
        
        # print(total_dis)

        if (total_diff and x_diff and y_diff and z_diff):
            pass
        
        else:
            print("ABORT -- Distance Error")
            print(total_dis, x_diff, y_diff, z_diff)
            # poshold(cf, 1, 0.2)
            cf.commander.send_stop_setpoint()
            cf.close_link()


def check_stab(cf):
    global drone1_stab
    global drone2_stab
    
    if (drone1_stab != (0,0,0,0) and drone2_stab != (0,0,0,0)):
        # total_dis = sqrt((drone1_stab[0]-drone2_stab[0])**2 + (drone1_stab[1]-drone2_stab[1])**2 + (drone1_stab[2]-drone2_stab[2])**2)
        
        roll_diff = 0 < abs(drone1_stab[0] - drone2_stab[0]) <= 9.426493884 
        pitch_diff = 0 < abs(drone1_stab[1] - drone2_stab[1])<= 30.9468110715
        yaw_diff = 0 < abs(drone1_stab[2] - drone2_stab[2]) <= 357.4603576
        # thrust_diff = 0.3256810903 < drone1_stab[3] - drone2_stab[3] < 1.6746508871
        
        if (roll_diff and pitch_diff and yaw_diff):
            pass
        
        else:
            print("ABORT -- Stabilizer Error")
            print(roll_diff, pitch_diff, yaw_diff)
            # poshold(cf, 1, 0.2)
            cf.commander.send_stop_setpoint()
            cf.close_link()


def position_callback(timestamp, data, logconf):
    # for i in data:
    #     key = i + "_" + logconf.cf.link_uri
    #     # print(key)
    #     if key in value_dict:
    #         value_dict[key].append(data[i])
    #     else:
    #         value_dict[key] = []

    global drone1_stab
    global drone2_stab
    global drone1_pos
    global drone2_pos
    # print(logconf)
    if 'kalman.stateX' in data:
        # pos = (data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'])
        if logconf.cf.link_uri == "radio://0/80/2M/E7E7E7E702":
            drone1_pos = (data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'])
    
        elif logconf.cf.link_uri == "radio://0/80/2M/E7E7E7E7E7":
            drone2_pos = (data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'])

        else:
            pass

    if 'stabilizer.roll' in data:
        if logconf.cf.link_uri == "radio://0/80/2M/E7E7E7E702":
            drone1_stab = (data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw'], data['stabilizer.thrust'])
    
        elif logconf.cf.link_uri == "radio://0/80/2M/E7E7E7E7E7":
            drone2_stab = (data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw'], data['stabilizer.thrust'])

        else:
            pass



        # print(drone1_pos)
        # print(drone2_pos)

    check_loc_dist(logconf.cf)
    check_stab(logconf.cf)


def start_position_printing(scf):
    log_blocks = {  'pos':['kalman.stateX', 'kalman.stateY', 'kalman.stateZ'],
                    'stab':['stabilizer.roll', 'stabilizer.pitch', 'stabilizer.yaw', 'stabilizer.thrust'],
                    'baro':['baro.asl', 'baro.temp', 'baro.pressure'],
                    # 'motor':['motor.m1', 'motor.m2', 'motor.m3', 'motor.m4'],
                }
    for i in log_blocks:
        # var = i
        var = LogConfig(name=i, period_in_ms=10)
        for j in log_blocks[i]:
            var.add_variable(j, 'float')
            # df[j] = []
            # value_dict[j] = []

        scf.cf.log.add_config(var)
        var.data_received_cb.add_callback(position_callback)
        var.start()


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    # time.sleep(2)
    wait_for_position_estimator(scf)


def poshold(cf, t, z):
    steps = t * 10

    for r in range(steps):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)


def pos_orient(cf, t, yaw, z):
    steps = t * 10

    for r in range(steps):
        cf.commander.send_hover_setpoint(0, 0, yaw, z)
        time.sleep(0.1)


def run_sequence(scf, params):
    cf = scf.cf

    # Number of setpoints sent per second
    fs = 4
    fsi = 1.0 / fs

    # Compensation for unknown error :-(
    comp = 1.3     #1.3

    # Base altitude in meters
    base = 0.15

    d = params['d']
    z = params['z']
    yaw = params['yaw']

    # poshold(cf, 2, base)

    ramp = fs * 2
    # for r in range(ramp):
    #     cf.commander.send_hover_setpoint(0, 0, 0, base + r * (z - base) / ramp)
    #     time.sleep(fsi)

    poshold(cf, 3, z)

    pos_orient(cf, 3, yaw, z)

    poshold(cf, 2, z)

    for _ in range(2):
        # The time for one revolution
        circle_time = 8

        steps = circle_time * fs
        for _ in range(steps):
            cf.commander.send_hover_setpoint(d * comp * math.pi / circle_time,
                                             0, 360.0 / circle_time, z)
            time.sleep(fsi)

    poshold(cf, 2, z)

    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0,
                                         base + (ramp - r) * (z - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 1, base)

    cf.commander.send_stop_setpoint()


# def activate_high_level_commander(scf):
#     scf.cf.param.set_value('commander.enHighLevel', '1')

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        # THIS MIGHT CAUSE ISSUES DEBUG THIS FIRST!!!!!!!!
        # swarm.parallel_safe(activate_high_level_commander)


        swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(start_position_printing)
        time.sleep(3)

        # time.sleep(5)

        swarm.parallel_safe(run_sequence, args_dict=params)
        # time.sleep(5)
        # print_df()