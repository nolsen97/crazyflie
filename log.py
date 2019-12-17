
import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import pandas as pd


f = open("logging.csv", 'w')

df = pd.DataFrame()
value_dict = {}

def print_df():
    for i in value_dict:
        # print(value_dict[i])
        df[i] = pd.Series(value_dict[i])
    df.to_csv(path_or_buf=f)

def log_data(timestamp, data, logconf):
    for i in data:
        value_dict[i].append(data[i])
    

def log_vars(scf):

    log_blocks = {  'pos':['kalman.stateX', 'kalman.stateY', 'kalman.stateZ'],
                    'stab':['stabilizer.roll', 'stabilizer.pitch', 'stabilizer.yaw', 'stabilizer.thrust'],
                    'baro':['baro.asl', 'baro.temp', 'baro.pressure'],
                    'motor':['motor.m1', 'motor.m2', 'motor.m3', 'motor.m4'],
                    'range1':['ranging.distance0', 'ranging.distance1', 'ranging.distance2', 'ranging.distance3'],
                    'range2':['ranging.distance4', 'ranging.distance5', 'ranging.distance6', 'ranging.distance7'],
                    'tdoa1':['tdoa.cc0', 'tdoa.cc1', 'tdoa.cc2', 'tdoa.cc3'],
                    'tdoa2':['tdoa.cc4', 'tdoa.cc5', 'tdoa.cc6', 'tdoa.cc7'],
                    'stateEstimate':['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'],

    }
    
    # for i in log_blocks:
    #     for j in log_blocks[i]:
    #         df[j] = []
    
    for i in log_blocks:
        # var = i
        var = LogConfig(name=i, period_in_ms=10)
        for j in log_blocks[i]:
            var.add_variable(j, 'float')
            # df[j] = []
            value_dict[j] = []

        scf.cf.log.add_config(var)
        var.data_received_cb.add_callback(log_data)
        var.start()
