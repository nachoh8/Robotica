class DebugBlockPi(object):
        def __init__(self):
            self.PORT_1 = 0x01
            self.PORT_2 = 0x02
            self.PORT_3 = 0x04
            self.PORT_4 = 0x08

            self.PORT_A = 0x01
            self.PORT_B = 0x02
            self.PORT_C = 0x04
            self.PORT_D = 0x08

        def set_motor_dps(self, motor_port_left, speed_dps_left):
            """
            Fake function to set motors dps and avoid execution errors
            :param motor_port_left: Not used
            :param speed_dps_left: Not used
            """
            pass

        def reset_all(self):
            """
            Fake function to reset motors and avoid execution errors
            """
            pass

        def set_sensor_type(self, a, b):
            pass