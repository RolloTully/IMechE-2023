import pymavlink
from pymavlink import mavutil
import time
from multiprocessing import Process
class Link(object):
    def __init__(self, address):
        self.connection = mavutil.mavlink_connection(address)
        print("Connection opened")
        self.heart_task = Process(target = self.send_heartbeat)
        slef.heart_task.start()
        #self.connection.request_data_stream_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 115200, 1)
    def send_heartbeat(self):
        while True:
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    def get_wind(self):
        return self.connection.messages['WIND_COV']
    def get_status(self):
        return self.connection.messages['MAV_LANDED_STAT']
    def get_gps(self):
        return self.connection.messages['GPS_RAW_INT']
    def is_armed(self):
        return self.connection.messages['MAV_MODE']
    def is_failsafe(self):
        return self.connection.messages['HL_FAILURE_FLAG']
    def arm(self):
        '''Arms drone unless a saftey check is not passed'''
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,1,0,0,0,0,0,0)
        return self.connection.recv.match(type='COMMAND_ACK', blocking = True)
    def disarm(self):
        '''Force disarms drone'''
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,0,21196,0,0,0,0,0)
        return self.connection.recv.match(type='COMMAND_ACK', blocking = True)
    def is_landed(self):
        return self.connection.messages['MAV_LANDED_STATE']
    def set_course(self, items):
        '''Uploads a new waypoint mission'''
        self.connection.mav.send()
        self.connection.recv.match(type='MISSION_REQUEST_INT', blocking = True)
        for item in items:
            self.connection.mav.send()
            self.connection.recv.match(type='COMMAND_ACK', blocking = True)


        pass
    def set_geofence(self):
        pass



class main():
    def __init__(self):
        self.link = Link('tcp:127.0.0.1:5760')
        self.state = ''
        self.task1 = Process(target = self.mainloop)
        self.task1.start()
        self.task2 = Process(target = self.check_state)
        #self.task2.start()

    def terminate(self):
        self.link.disarm()

    def failsafe(self, type):
        if type == 0: #failsafe while grounded
            self.link.disarm()
        elif type == 1: #failsafe while in air, FTS
            self.terminate()

    def check_state(self):
        while self.link.is_landed():
            self.link.get_status()
    def mainloop(self):
        print(self.link.connection.recv_match(type="SYS_STATUS", blocking = True))

if __name__ == "__main__":
    main()
