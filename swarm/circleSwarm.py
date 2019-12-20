"""
A script to fly 5 Crazyflies in formation. One stays in the center and the
other four fly aound it in a circle. Mainly intended to be used with the
Flow deck.
The starting positions are vital and should be oriented like this



v1        v0



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

value_dict = {}

# Change uris according to your setup
URI0 = 'radio://0/80/2M/E7E7E7E7E7'
URI1 = 'radio://0/80/2M/E7E7E7E702'

# d: diameter of circle
# z: altituce
# yaw: how the drone should orient itself after take off
params0 = {'d': 1.0, 'z': 0.5, 'yaw':0.0}
params1 = {'d': 1.0, 'z': 0.5, 'yaw':60.0}


uris = {
    URI0,
    URI1,
}

params = {
    URI0: [params0],
    URI1: [params1],
}

def check_loc_dist(cf):
    global drone1_pos
    global drone2_pos

    if (drone1_pos != (0,0,0) and drone2_pos != (0,0,0)):
        total_dis = math.sqrt((drone1_pos[0]-drone2_pos[0])**2 + (drone1_pos[1]-drone2_pos[1])**2 + (drone1_pos[2]-drone2_pos[2])**2)
        total_diff = 0.1 < total_dis < 1.8
        x_diff = 0 < abs(drone1_pos[0] - drone2_pos[0]) <= 1.5
        y_diff = 0 < abs(drone1_pos[1] - drone2_pos[1]) <= 1.8
        z_diff = 0 < abs(drone1_pos[2] - drone2_pos[2]) <= 0.7435006574

        if (total_diff and x_diff and y_diff and z_diff):
            pass

        else:
            print("ABORT -- Distance Error")
            print(total_dis, x_diff, y_diff, z_diff)
            cf.commander.send_stop_setpoint()
            cf.close_link()


def check_stab(cf):
    global drone1_stab
    global drone2_stab

    if (drone1_stab != (0,0,0,0) and drone2_stab != (0,0,0,0)):

        roll_diff = 0 < abs(drone1_stab[0] - drone2_stab[0]) <= 9.426493884
        pitch_diff = 0 < abs(drone1_stab[1] - drone2_stab[1])<= 30.9468110715
        yaw_diff = 0 < abs(drone1_stab[2] - drone2_stab[2]) <= 357.4603576

        if (roll_diff and pitch_diff and yaw_diff):
            pass

        else:
            print("ABORT -- Stabilizer Error")
            print(roll_diff, pitch_diff, yaw_diff)
            cf.commander.send_stop_setpoint()
            cf.close_link()


def position_callback(timestamp, data, logconf):

    global drone1_stab
    global drone2_stab
    global drone1_pos
    global drone2_pos

    if 'kalman.stateX' in data:
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

    check_loc_dist(logconf.cf)
    check_stab(logconf.cf)


def start_position_printing(scf):
    log_blocks = {  'pos':['kalman.stateX', 'kalman.stateY', 'kalman.stateZ'],
                    'stab':['stabilizer.roll', 'stabilizer.pitch', 'stabilizer.yaw', 'stabilizer.thrust'],
                    'baro':['baro.asl', 'baro.temp', 'baro.pressure'],
                }

    for i in log_blocks:
        var = LogConfig(name=i, period_in_ms=10)
        for j in log_blocks[i]:
            var.add_variable(j, 'float')

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

    # Compensation
    comp = 1.3

    # Base altitude in meters
    base = 0.15

    d = params['d']
    z = params['z']
    yaw = params['yaw']


    ramp = fs * 2

    poshold(cf, 3, z)

    pos_orient(cf, 3, yaw, z)

    poshold(cf, 2, z)

    num_revolutions = 2
    for _ in range(num_revolutions):

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


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(start_position_printing)
        time.sleep(3)

        swarm.parallel_safe(run_sequence, args_dict=params)
