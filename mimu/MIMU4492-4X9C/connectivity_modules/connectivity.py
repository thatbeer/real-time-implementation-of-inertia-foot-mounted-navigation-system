import abc


class Connectivity(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def __init__(self):
        pass

    @abc.abstractmethod
    def open(self, device_params):
        pass

    @abc.abstractmethod
    def send(self, data):
        pass

    @abc.abstractmethod
    def receive(self, len):
        pass

    @abc.abstractmethod
    def close(self):
        pass

    @abc.abstractmethod
    def input_validation(self, *kargs):
        pass

    # @staticmethod
    # def get_connectivity(conn_type):
    #     if str(conn_type).upper() == 'WIFI':
    #         return WifiConnectivity()
    #     elif str(conn_type).upper() == 'BLE':
    #         return BleConnectivity()
    #     elif str(conn_type).upper() == 'USB':
    #         return UsbConnectivity()
    #     return None
