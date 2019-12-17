
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

    # print(log_blocks)



    # log_pos = LogConfig(name='Position', period_in_ms=10)
    # log_pos.add_variable('kalman.stateX', 'float')
    # log_pos.add_variable('kalman.stateY', 'float')
    # log_pos.add_variable('kalman.stateZ', 'float')

    # log_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    # log_stab.add_variable('stabilizer.roll', 'float')
    # log_stab.add_variable('stabilizer.pitch', 'float')
    # log_stab.add_variable('stabilizer.yaw', 'float')
    # log_stab.add_variable('stabilizer.thrust', 'float')

    # log_baro = LogConfig(name='Baro', period_in_ms=10)
    # log_baro.add_variable('baro.asl', 'float')
    # log_baro.add_variable('baro.temp', 'float')
    # log_baro.add_variable('baro.pressure', 'float')

    # log_motor = LogConfig(name="Motor", period_in_ms=10)
    # log_motor.add_variable('motor.m1', 'int')
    # log_motor.add_variable('motor.m2', 'int')
    # log_motor.add_variable('motor.m3', 'int')
    # log_motor.add_variable('motor.m4', 'int')

    # log_range1 = LogConfig(name="Range1", period_in_ms=10)
    # log_range1.add_variable('ranging.distance0', 'float')
    # log_range1.add_variable('ranging.distance1', 'float')
    # log_range1.add_variable('ranging.distance2', 'float')
    # log_range1.add_variable('ranging.distance3', 'float')

    # log_range2 = LogConfig(name="Range2", period_in_ms=10)
    # log_range2.add_variable('ranging.distance4', 'float')
    # log_range2.add_variable('ranging.distance5', 'float')
    # log_range2.add_variable('ranging.distance6', 'float')
    # log_range2.add_variable('ranging.distance7', 'float')

    # log_cc1 = LogConfig(name="cc1", period_in_ms=10)
    # log_cc1.add_variable('ranging.distance0', 'float')
    # log_cc1.add_variable('ranging.distance1', 'float')
    # log_cc1.add_variable('ranging.distance2', 'float')
    # log_cc1.add_variable('ranging.distance3', 'float')

    # log_range2 = LogConfig(name="Range2", period_in_ms=10)
    # log_range2.add_variable('ranging.distance4', 'float')
    # log_range2.add_variable('ranging.distance5', 'float')
    # log_range2.add_variable('ranging.distance6', 'float')
    # log_range2.add_variable('ranging.distance7', 'float')

    # log_stateEstimate = LogConfig(name="StateEstimate", period_in_ms=10)
    # log_stateEstimate.add_variable('stateEstimate.x', 'float')
    # log_stateEstimate.add_variable('stateEstimate.y', 'float')
    # log_stateEstimate.add_variable('stateEstimate.z', 'float')

    # scf.cf.log.add_config(log_pos)
    # log_pos.data_received_cb.add_callback(log_data)
    
    # scf.cf.log.add_config(log_stab)
    # log_stab.data_received_cb.add_callback(log_data)
    
    # scf.cf.log.add_config(log_baro)
    # log_baro.data_received_cb.add_callback(log_data)
    
    # scf.cf.log.add_config(log_motor)
    # log_motor.data_received_cb.add_callback(log_data)

    # scf.cf.log.add_config(log_range1)
    # log_range1.data_received_cb.add_callback(log_data)

    # scf.cf.log.add_config(log_range2)
    # log_range2.data_received_cb.add_callback(log_data)

    # log_stab.start()
    # log_pos.start()
    # log_baro.start()
    # log_motor.start()
    # log_range1.start()
    # log_range2.start()
    # log_stateEstimate.start()