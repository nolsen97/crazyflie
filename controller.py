import logging
import time
import sys
from threading import Thread
import curses
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

f = open("alt_log.txt", 'w')

def ardu_map(x, old_min, old_max, new_min, new_max):
    return ((x - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min

class Hover:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        self._status = {}
        self._status['alt'] = 5*[0]
        self._status['pitch'] = 0
        self._status['roll'] = 0
        self._status['yaw'] = 0

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='stablizer', period_in_ms=10)
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('baro.asl', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        Thread(target=self._hover).start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        self._status['alt'].pop(0)
        self._status['alt'].append(data['baro.asl'])
        self._status['pitch'] = data['stabilizer.pitch']
        self._status['roll'] = data['stabilizer.roll']
        self._status['yaw'] = data['stabilizer.yaw']

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _hover(self):
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(1)
        stdscr.nodelay(1)

        stdscr.refresh()

        targetAlt = 254.0
        target_pitch, target_roll, = 0, 0


        #run_time = 10

        thrust = 20000
        max_thrust = 40000
        roll = 0
        pitch = 0
        yaw = 0

        # Unlock startup thrust protection.
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Turn on altitude hold.
        self._cf.param.set_value("flightmode.althold","True")
        while True:
            self._cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            time.sleep(0.01)
            currentAlt = sum(self._status['alt'])/5
            print(currentAlt)
            if (currentAlt < targetAlt and thrust < max_thrust):
                thrust += 500
            if currentAlt >= targetAlt:
                print("takeoff complete")
                #thrust = 32568
                break

        #while time.time() <= start_time + run_time:
        start_time = time.time()
        key = ''
        while key != ord('q'):
            key = stdscr.getch()
            stdscr.refresh()
            # Update the position.
            self._cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            time.sleep(0.01)

            alt_error = targetAlt - (sum(self._status['alt'])/5)
            end_time = time.time()
            unmapped_thrust = alt_error*25.0 + (alt_error/(end_time-start_time))*0.5
            start_time = time.time()
            if unmapped_thrust > 100:
                unmapped_thrust = 100
            if unmapped_thrust < -100:
                unmapped_thrust = -100
            thrust = int(ardu_map(unmapped_thrust, -100, 100, 10001, 60000))

            #pitch_error = target_pitch - self._status['pitch']
            #pitch = pitch_error
            if key == curses.KEY_UP:
                pitch += 0.1
            elif key == curses.KEY_DOWN:
                pitch -= 0.1
            elif key == curses.KEY_LEFT:
                roll -= 0.1
            elif key == curses.KEY_RIGHT:
                roll += 0.1
            else:
                pitch = pitch
                roll = roll
            #roll_error = target_roll - self._status['roll']
            #roll = roll_error

            yaw = 0

            f.write(str(sum(self._status['alt'])/5) + "\n")
            print(str(sum(self._status['alt'])/5) + "\n")  # thrust, pitch, roll, yaw)

        curses.endwin()
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = Hover(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
    f.close()
