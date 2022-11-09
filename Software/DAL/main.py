import pymavlink
from pymavlink import mavutil
import time
from multiprocessing import Process, active_children
from time import sleep
import signal
import sys
import json

class Link(object):
    def __init__(self, address):
        self.connection = mavutil.mavlink_connection(address)
        print("Connection Established")
        self.connection.wait_heartbeat()
        self.connection.mav.ping_send(int(time.time() * 1e6),0,0,0)
        self.heart_task = Process(target = self.send_heartbeat)
        self.heart_task.start()
    def send_heartbeat(self):
        while True:
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
            self.connection.wait_heartbeat()
            sleep(1)
    def Update_Variables(self):
        self.message_package = self.connection.messages
    def get_wind(self):
        return self.connection.messages['WIND']
    def get_status(self):
        return self.connection.messages['SYS_STATUS']
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

class Waypoint(object):
    def __init__(self, latitude = None, longditude = None, radius= 30, altitude = 60, flag = None, cont= None):
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.radius = radius         #Way point acceptance radius
        self.altitude = altitude     #Target waypoint altitude
        self.flag = flag             #Waypoint flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class DAL(object):
    '''The Dynamic Auto Landing mission item instructs the flight director to perform the relevent actions'''
    def __init__(self, latitude = None, longditude = None, radius= 30, altitude = 60, flag = None):
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.radius = 10             #Landing target radius
        self.altitude = altitude     #Landing waypoint altitude
        self.flag = "DAL"            #DAL flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class PCR(object):
    '''This mission item in formed the flight director to monitor the aircraft position and release the cargo at the optimal moment'''
    def __init__(self, latitude = None, longditude = None, radius= 30, altitude = 60, flag = None):
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.radius = radius         #Way point acceptance radius
        self.altitude = altitude     #Target waypoint altitude
        self.flag = "PCR"             #Waypoint flag, Precision Cargo Release flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class Mission(object):
    def __init__(self):
        '''The Mission path is composed of an array of objects that represent specific mission elements such as cargo release, DAL, and mission waypoints'''
        self.path = None
        self.mission_index = 0
    def next_mission_item(self):
        self.mission_index += 1
        return self.path[self.mission_index]

class main():
    def __init__(self):

        self.link = Link("/dev/ttyACM0")


        signal.signal(signal.SIGINT, self.keyboard_interupt_handler)
        self.FailSafe_Process = Process(target = self.Failsafe_Watchdog)
        self.mainloop_process = Process(target = self.mainloop)
        self.FailSafe_Process.start()
        self.mainloop_process.start()

    def Heuristic_Automatic_Landing_Operation(self):# HALO
        '''A fully autinimious heuristic algorithum that optimised the landing heading and flight path angle'''

    def Precision_Cargo_Release(self):# PCR
        '''Uses onboard velocity time and position estimations to accuratly release the cargo'''

    def Load_mission(self):
        '''Loads in mission waypoint parameters from a JSON file on an sd card'''
        self.mission_file = json.load("mission.json")
        self.mission = Mission()

    def Failsafe_Watchdog(self):
        '''Monitors system parameters to ensure complicate with all failsafe conditions'''
        '''This method also enacts the failsafe protocol when nessecary to maintain the saftey of ground staff'''
        '''
        This method must monitor:
        The failsafe watchdog to ensure proper functioning, is the failsafe watchdog running sufficently fast?
        geofence compliance
        minimum altitude compliance
        maximum speed compliance
        battery voltage compliance
        minimum speed compliance
        '''
        self.FS_Maximum_Period = 1 #second
        self.Velocity_FS_Threshold = [0,30]# Meters per second, keeps the done in line with competiton rules
        self.Altitude_FS_Threshold = [0,120]#Above altitude, keeps the drone in line with regulations
        self.Battery_Voltage_FS_Threshold = [0,24]#Volts, prevents brown outs
        self.Battery_Current_FS_Threshold = [0,100]#Amps, prevents catastrophic short circuit
        while True:
            sleep(10)
            pass



    def keyboard_interupt_handler(self):
        '''Execute order 66, kills all the children'''
        '''Interupts the standard keyboard interupt to ensure all processes are correctly terminated.'''
        if not self.link.is_landed():
            print("Craft in flight, command override active.")
        else:
            self.link.disarm()
            self.active = active_children()
            print(len(self.active),"children found.")
            for self.child in self.active:
                self.child.kill()
            print("All child processes terminated.")
            print("Terminating flight director.")
            sys.exit()


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
        while True:
            #print(self.link.is_armed())
            print(self.link.connection.messages['WIND'])
            print(self.link.connection.messages['AHRS'])
            print(self.link.connection.messages['GLOBAL_POSITION_INT'])



if __name__ == "__main__":

    main()
