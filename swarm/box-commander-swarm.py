# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
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
Simple example of a swarm using the High level commander.

The swarm takes off and flies a synchronous square shape before landing.
The trajectories are relative to the starting positions and the Crazyfles can
be at any position on the floor when the script is started.

This example is intended to work with any absolute positioning system.
It aims at documenting how to use the High Level Commander together with
the Swarm class.
"""
import time
import math
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
import pandas as pd

f = open("box-swarm-log.csv", 'w')

df = pd.DataFrame()
value_dict = {}


drone1_pos = (0, 0, 0)
drone2_pos = (0, 0, 0)
drone1_stab = (0, 0, 0, 0)
drone2_stab = (0, 0, 0, 0)


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
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)

def check_loc_dist(cf):
    global drone1_pos
    global drone2_pos
    
    if (drone1_pos != (0,0,0) and drone2_pos != (0,0,0)):
        total_dis = math.sqrt((drone1_pos[0]-drone2_pos[0])**2 + (drone1_pos[1]-drone2_pos[1])**2 + (drone1_pos[2]-drone2_pos[2])**2)
        total_diff = 0.2542146091 < total_dis < 1.7571476809
        x_diff = 0 < abs(drone1_pos[0] - drone2_pos[0]) <= 0.52233231
        y_diff = 0.226760864 < abs(drone1_pos[1] - drone2_pos[1]) <= 1.609792709
        z_diff = 0 < abs(drone1_pos[2] - drone2_pos[2]) <= 1.1269430888
        
        if (total_diff and x_diff and y_diff and z_diff):
            pass
        
        else:
            print("ABORT -- Distance Error")
            # print(total_dis)
            print(total_diff, x_diff, y_diff, z_diff)

            commander = cf.high_level_commander
            commander.land(0.0, 2.0)
            time.sleep(2)

            commander.stop()

def check_stab(cf):
    global drone1_stab
    global drone2_stab
    
    if (drone1_stab != (0,0,0,0) and drone2_stab != (0,0,0,0)):
        # total_dis = sqrt((drone1_stab[0]-drone2_stab[0])**2 + (drone1_stab[1]-drone2_stab[1])**2 + (drone1_stab[2]-drone2_stab[2])**2)
        
        roll_diff = 0 < abs(drone1_stab[0] - drone2_stab[0]) <= 17.6666653797
        pitch_diff = 0 < abs(drone1_stab[1] - drone2_stab[1])<= 18.1477545689
        yaw_diff = 0 < abs(drone1_stab[2] - drone2_stab[2]) <= 27.524837013
        # thrust_diff = 0.3256810903 < drone1_stab[3] - drone2_stab[3] < 1.6746508871
        
        if (roll_diff and pitch_diff and yaw_diff):
            pass
        
        else:
            print("ABORT -- Stabilizer Error")
            # print(total_dis)
            print(roll_diff, pitch_diff, yaw_diff)
            commander = cf.high_level_commander
            commander.land(0.0, 2.0)
            time.sleep(2)

            commander.stop()

def position_callback(timestamp, data, logconf):
    # x = data['kalman.stateX']
    # y = data['kalman.stateY']
    # z = data['kalman.stateZ']
    # print('x:{}, y:{}, z:{}'.format(x, y, z))
    # print(data, logconf.cf.link_uri)
    
    # for i in data:
    #     key = i + "_" + logconf.cf.link_uri
    #     # print(key)
    #     if key in value_dict:
    #         value_dict[key].append(data[i])
    #     else:
    #         value_dict[key] = []
  
    global drone1_pos
    global drone2_pos
    global drone1_stab
    global drone2_stab
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


def run_shared_sequence(scf):
    # activate_mellinger_controller(scf, False)

    # run1: box_size: 0.5 1m apart, run2: 1.0 (swapped) 1m apart, run3: 0.5 (close start) 0.5m apart
    box_size = 1.0
    flight_time = 3

    commander = scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)


    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()


uris = {
    'radio://0/80/2M/E7E7E7E7E7',
    'radio://0/80/2M/E7E7E7E702',
    # 'radio://0/80/2M/E7E7E7E702',
    # Add more URIs if you want more copters in the swarm
}

def print_df():
    for i in value_dict:
        # print(value_dict[i])
        df[i] = pd.Series(value_dict[i])
    df.to_csv(path_or_buf=f)

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)
        time.sleep(5)
        swarm.parallel_safe(start_position_printing)
        swarm.parallel_safe(run_shared_sequence)
        """
        Try using parallel instead of parallel safe
        or sequential
        """
        # time.sleep(5)
        # print_df()