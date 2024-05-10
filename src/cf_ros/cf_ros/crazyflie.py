import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

from scipy.spatial.transform import Rotation as R
import numpy as np

from vicon_receiver.msg import Position

import time

# uri = uri_helper.uri_from_env(default='radio://0/120/2M/E7E7E7E709')
uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708')


class cf_publisher(Node):

    def __init__(self):
        # self.pos = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        # Connect some callbacks from the Crazyflie API
        super().__init__('crazyflie_publisher')
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % uri)

        # Try to connect to the Crazyflie
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf.open_link(uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True


        self.state_pos = np.array([0, 0, 0])
        self.state_quat = R.from_quat([1, 1, 1, 1])

        self.irobot_pos = np.array([0, 0, 0])

        self.cf_vicon_subscriber = self.create_subscription(Position, "vicon/ns_fly/ns_fly", self.cf_vicon_callback, 10)
        self.irobot_vicon_subscriber = self.create_subscription(Position, "vicon/ns_roomba/ns_roomba", self.irobot_vicon_callback, 10)
    
        self.t_p = time.time()
        self.e_i = 0
        self.e_p = 0

        self.I = False
 
    def cf_vicon_callback(self, msg_in):
        self.vicon_pos = np.array([msg_in.x_trans, msg_in.y_trans, msg_in.z_trans])/1000
        quat = np.array([msg_in.x_rot, msg_in.y_rot, msg_in.z_rot, msg_in.w])
        qnorm = np.linalg.norm(quat)
        self.vicon_yaw = 0
        if(not qnorm < 0.001):
            self.vicon_yaw = R.from_quat([msg_in.x_rot, msg_in.y_rot, msg_in.z_rot, msg_in.w]).as_euler('zyx')[0]

        self.vicon_yaw -= np.pi/2
        rot_mat = np.array([[np.cos(self.vicon_yaw), -np.sin(self.vicon_yaw)],[np.sin(self.vicon_yaw), np.cos(self.vicon_yaw)]])

        kpx = 0.4
        kpy = 0.2
        kp = np.array([[kpx, 0],[0, kpy]])

        error_vect = self.irobot_pos - self.vicon_pos

        kix = 0.0001
        kiy = 0.0001
        ki = np.array([[kix, 0],[0, kiy]])

        if not self.I:
            u = rot_mat @ (kp @ error_vect[:2]) 
            self.I = True
        else:
            dt = time.time() - self.t_p
            self.e_i += (error_vect + self.e_p) * dt * 0.5
            self.t_p = time.time()
            u = rot_mat @ (kp @ error_vect[:2] + ki @ self.e_i[:2]) 
        
        self.e_p = error_vect
        u = rot_mat @ (kp @ error_vect[:2])

        print(u)

        if u[0] > 1.0:
            u[0] = 1.0
        if u[1] > 1.0:
            u[0] = 1.0

        # print(np.degrees(self.vicon_yaw))       
        self.cf.commander.send_hover_setpoint(u[0], u[1], 0, 0.2)
        self.cf.commander.send_notify_setpoint_stop(remain_valid_milliseconds=10)
        # self.cf.commander.send_hover_setpoint(0, 0.1, 0, 0.2)   
        pass
        

    def irobot_vicon_callback(self, msg_in):
        self.irobot_pos = np.array([msg_in.x_trans, msg_in.y_trans, msg_in.z_trans])/1000

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self.lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self.lg_stab.add_variable('stateEstimate.x', 'float')
        self.lg_stab.add_variable('stateEstimate.y', 'float')
        self.lg_stab.add_variable('stateEstimate.z', 'float')
        self.lg_stab.add_variable('stabilizer.roll', 'float')
        self.lg_stab.add_variable('stabilizer.pitch', 'float')
        self.lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self.lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self.cf.log.add_config(self.lg_stab)
            # This callback will receive the data
            self.lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self.lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self.lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        msg_out= Pose()

        # print(data)

        msg_out.position.x = data['stateEstimate.x']
        msg_out.position.y = data['stateEstimate.y']
        msg_out.position.z = data['stateEstimate.z']

        yaw = data['stabilizer.yaw']
        pitch = data['stabilizer.pitch']
        roll = data['stabilizer.roll']

        self.yaw = yaw
        r = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)

        msg_out.orientation.x = r.as_quat()[0]
        msg_out.orientation.y = r.as_quat()[1]
        msg_out.orientation.z = r.as_quat()[2]
        msg_out.orientation.w = r.as_quat()[3]

        self.state_pos = np.array([data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']])
        self.state_quat = np.array([r.as_quat()[0], r.as_quat()[1], r.as_quat()[1], r.as_quat()[3]])

        # print("state", self.state_pos, self.state_quat)

        """Callback from a the log API when data arrives"""
        """print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()"""


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
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

def main(args=None):

    rclpy.init(args=args)
    pub = cf_publisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
