"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from log import *

f = open("logging2.csv", 'w')

df = pd.DataFrame()
value_dict = {}

# URI to the Crazyflie to connect to

uri = 'radio://0/80/2M/E7E7E7E7E7'
# uri = 'radio://0/90/2M/E7E7E7E701'
#uri = 'radio://0/80/2M/E7E7E7E702'

# Change the sequence according to your setup
#             x    y    z  YAW
#position = (0.5, 0.5, 0.5, 0)
position = (1.20, 2.55, 1.0, 0.0)

        
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

            #print("{} {} {}".
            #    format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    # x = data['kalman.stateX']
    # y = data['kalman.stateY']
    # z = data['kalman.stateZ']
    # print('x:{}, y:{}, z:{}'.format(x, y, z))
    # print(data)
    for i in data:
        if i in value_dict:
            value_dict[i].append(data[i])
        else:
            value_dict[i] = []

def start_position_printing(scf):
    # print("here")
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    log_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    log_stab.add_variable('stabilizer.roll', 'float')
    log_stab.add_variable('stabilizer.pitch', 'float')
    log_stab.add_variable('stabilizer.yaw', 'float')
    log_stab.add_variable('stabilizer.thrust', 'float')

    scf.cf.log.add_config(log_conf)
    scf.cf.log.add_config(log_stab)
    log_conf.data_received_cb.add_callback(position_callback)
    log_stab.data_received_cb.add_callback(position_callback)
    log_conf.start()
    log_stab.start()
    # LoggingExample(uri)


def run_sequence(scf, position):
    cf = scf.cf
    end = time.time() + 3
    print('Setting position {}'.format(position))
    
    while time.time() < end:
        cf.commander.send_position_setpoint(position[0],
                                            position[1],
                                            position[2],
                                            position[3])
        time.sleep(0.01)
        #position = (position[0], position[1], position[2], position[3]+10)

    for i in range(30):
        cf.commander.send_position_setpoint(position[0], position[1], 0, 0.0)
        time.sleep(0.1)
    
    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        start_position_printing(scf)

        # log_vars(scf)
        run_sequence(scf, position)
        # print_df()

    for i in value_dict:
        # print(value_dict[i])
        df[i] = pd.Series(value_dict[i])
    df.to_csv(path_or_buf=f)
