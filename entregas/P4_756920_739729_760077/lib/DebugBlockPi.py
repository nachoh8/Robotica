
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

    def set_motor_dps(self, port, dps):
        pass

    def reset_all(self):
        pass

    def set_sensor_type(self, port, type):
        pass

    def get_motor_encoder(self, port) -> int:
        return 0
    
    def offset_motor_encoder(self, port, encoder):
        pass